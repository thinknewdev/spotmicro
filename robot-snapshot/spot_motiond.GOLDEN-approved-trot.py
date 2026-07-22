#!/usr/bin/env python3
"""SpotMicro motion daemon — owns all servo/IMU I2C, runs the proven
stand/walk/balance code as a state machine driven by a command file.

States: REST (released) <-> STAND (balance hold) <-> WALK (N cycles -> STAND)
Commands (JSON written to /tmp/spot_cmd): {"cmd": "stand"|"walk"|"rest", "cycles": 8}
Status: written to /tmp/spot_status.json every ~0.5s.
Kill switch: touch /tmp/spot_stop -> fold + exit.

Calibration source of truth: ~/spotmicroai.json (channels FIXED 2026-07-05)
+ standing pose from ~/spotmicroai/robot_calibration.json.
"""
import json, time, subprocess, sys, math, os, statistics, signal

import busio
from board import SCL, SDA
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo as af_servo
from mpu6050 import mpu6050

CMD_FILE = "/tmp/spot_cmd"
STATUS_FILE = "/tmp/spot_status.json"
STOP_FILE = "/tmp/spot_stop"
UV = 0x1
HZ = 20
DT = 1.0 / HZ

# ---- calibrated pose + gait (2026-07-05 session, user-approved) ----
CAL = json.load(open(os.path.expanduser("~/spotmicroai/robot_calibration.json")))
STANDING = {k: v for k, v in CAL["standing_pose"].items() if not k.startswith("_")}
ARM = ["arm_rotation", "arm_lift", "arm_range", "arm_cam_tilt"]
THIGHS = {"front_leg_left": -1, "front_leg_right": +1, "rear_leg_left": -1, "rear_leg_right": +1}
KNEES = {"front_feet_left": -1, "front_feet_right": +1, "rear_feet_left": -1, "rear_feet_right": +1}
LEG_OF_KNEE = {"front_feet_left": "front_leg_left", "front_feet_right": "front_leg_right",
               "rear_feet_left": "rear_leg_left", "rear_feet_right": "rear_leg_right"}
PAIR_A = ["front_leg_left", "rear_leg_right"]
PAIR_B = ["front_leg_right", "rear_leg_left"]
LEGS = list(THIGHS)
SWING_FWD, SWING_BACK, LIFT, CYCLE_T = 13.0, 17.0, 26.0, 2.0
LIFT_BIAS = {"rear_feet_right": 10.0}  # this foot drags; give it extra clearance
WALK_SWAY = 3.5    # rhythmic weight shift off the swinging diagonal (fluidity + clearance)
ARM_BOB = 4.0      # subtle arm_lift bob synced to stride
FWD_BIAS = 0.0     # disabled — destabilized the walk (kept for tuning)
                   # walk-time CoM-forward shift: rear lifts got big enough to
                   # risk a backward tip; keeping weight forward prevents it
PUSH_EXT = 11.0  # knee extension through stance = horizontal foot drive (propulsion)
TURN_GAIN = 2.0  # steering: inner side 1-G*|turn| -> full turn = inner strides BACKWARD (pivot)

# SIT: rear legs tucked (butt grounded), front legs brace the raised chest.
SIT_POSE = {
    "rear_leg_left": 168, "rear_leg_right": 12,
    "rear_feet_left": 25, "rear_feet_right": 155,
    "front_leg_left": 120, "front_leg_right": 60,
    "front_feet_left": 107, "front_feet_right": 103,
    "rear_shoulder_left": 90, "rear_shoulder_right": 90,
    "front_shoulder_left": 108, "front_shoulder_right": 108,
}
WAVE_THIGH_UP = 82      # front_leg_right raised for the wave
WAVE_KNEE_A, WAVE_KNEE_B = 100, 135   # paw wag endpoints
WAVE_WAGS = 3
KP, DEADBAND, SLEW, OFFSET_CLAMP = 0.15, 4.0, 0.9, 8.0
LEG_LIMITS = {"rear_leg_left": (80, 175), "front_leg_left": (80, 175),
              "rear_leg_right": (5, 100), "front_leg_right": (5, 100)}

cfg = json.load(open(os.path.expanduser("~/spotmicroai.json")))
sc = {}
for g in cfg["motion_controller"]:
    if "servos" in g:
        for sd in g["servos"]:
            for n, a in sd.items(): sc[n] = a[0]

i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c, address=0x40, reference_clock_speed=25000000)
pca.frequency = 50
mpu = mpu6050(0x68)

servos = {}
for n in list(STANDING) + ARM:
    s = sc[n]
    o = af_servo.Servo(pca.channels[s["channel"]])
    o.set_pulse_width_range(min_pulse=s["min_pulse"], max_pulse=s["max_pulse"])
    servos[n] = o

rest = {n: sc[n]["rest_angle"] for n in servos}
for k in ("front_shoulder_left", "front_shoulder_right", "rear_shoulder_left", "rear_shoulder_right"):
    rest[k] = STANDING[k]   # shoulders never rotate
cur = {}
state = "REST"


def log(m): print(f"[{time.strftime('%H:%M:%S')}] {m}", flush=True)


def throttled():
    out = subprocess.run(["vcgencmd", "get_throttled"], capture_output=True, text=True).stdout
    return int(out.strip().split("=")[1], 16)


def tilt():
    rs, ps = [], []
    for _ in range(3):
        for attempt in range(3):
            try:
                a = mpu.get_accel_data(); break
            except OSError:
                if attempt == 2: raise
                time.sleep(0.005)
        rs.append(math.atan2(a["y"], a["z"]) * 57.2958)
        ps.append(math.atan2(-a["x"], math.sqrt(a["y"]**2 + a["z"]**2)) * 57.2958)
    return statistics.median(rs), statistics.median(ps)


def apply(pose):
    for n, a in pose.items():
        v = max(0, min(180, a))
        for attempt in range(3):
            try:
                servos[n].angle = v; break
            except OSError:
                if attempt == 2: raise
                time.sleep(0.005)
        cur[n] = v


def write_status(extra=None):
    try:
        r, p = tilt()
    except Exception:
        r = p = None
    d = {"state": state, "roll": r, "pitch": p, "throttled": hex(throttled()), "ts": time.time()}
    if extra: d.update(extra)
    tmp = STATUS_FILE + ".tmp"
    with open(tmp, "w") as f: json.dump(d, f)
    os.replace(tmp, STATUS_FILE)


def read_cmd():
    """Return new command dict or None. Consumes the file."""
    if not os.path.exists(CMD_FILE): return None
    try:
        with open(CMD_FILE) as f: c = json.load(f)
    except Exception:
        c = None
    try: os.remove(CMD_FILE)
    except OSError: pass
    return c


def fold_to_rest():
    global state
    if not cur:   # never energized
        state = "REST"; return
    log("folding to crouch (eased, ~3s)...")
    start = dict(cur)
    STEPS = 40
    for step in range(1, STEPS + 1):
        f = 0.5 - 0.5 * math.cos(math.pi * step / STEPS)   # ease-in-out: no jerk
        apply({n: start[n] + (rest[n] - start[n]) * f for n in start})
        time.sleep(0.07)
    time.sleep(0.3)
    for ch in range(16): pca.channels[ch].duty_cycle = 0
    cur.clear()
    state = "REST"
    log("REST (servos released)")


def stand_up():
    """REST -> STAND via proven staggered energize + 18-slice pop."""
    global state
    log("standing up...")
    for n in servos:
        apply({n: rest[n]}); time.sleep(0.05)
        if throttled() & UV: fold_to_rest(); return False
    time.sleep(0.3)
    start = dict(cur)
    target = dict(STANDING, **{n: rest[n] for n in ARM})
    for step in range(1, 19):
        f = step / 18.0
        apply({n: start[n] + (target[n] - start[n]) * f for n in target})
        if throttled() & UV:
            time.sleep(1.5)
            if throttled() & UV: fold_to_rest(); return False
        time.sleep(0.08)
    time.sleep(1.0)
    state = "STAND"
    log("STAND")
    return True


def ease_to(pose, seconds=2.5):
    """Eased transition from current pose to target (cosine, no jerk)."""
    start = dict(cur)
    target = dict(pose)
    steps = max(10, int(seconds / 0.07))
    for step in range(1, steps + 1):
        f = 0.5 - 0.5 * math.cos(math.pi * step / steps)
        apply({n: start.get(n, target[n]) + (target[n] - start.get(n, target[n])) * f for n in target})
        time.sleep(0.07)


def do_sit():
    global state
    log("SIT: tucking rear, bracing front...")
    ease_to(dict(SIT_POSE, **{n: rest[n] for n in ARM}), seconds=2.5)
    state = "SIT"
    log("SIT")


def do_wave():
    """From SIT: raise front-right leg and wag the paw."""
    global state
    log("WAVE: raising front-right paw...")
    ease_to({"front_leg_right": WAVE_THIGH_UP, "front_feet_right": WAVE_KNEE_A}, seconds=1.0)
    for _ in range(WAVE_WAGS):
        ease_to({"front_feet_right": WAVE_KNEE_B}, seconds=0.45)
        ease_to({"front_feet_right": WAVE_KNEE_A}, seconds=0.45)
    log("WAVE: paw back down")
    ease_to({"front_leg_right": SIT_POSE["front_leg_right"],
             "front_feet_right": SIT_POSE["front_feet_right"]}, seconds=1.0)
    state = "SIT"


# ================= IK-based crawl (proper legs) =================
L1_MM, L2_MM = 110.0, 135.0   # user-measured segments

LEG_META = {  # leg -> (side, thigh servo, knee servo)
    "rear_left":  ("left",  "rear_leg_left",  "rear_feet_left"),
    "front_left": ("left",  "front_leg_left", "front_feet_left"),
    "rear_right": ("right", "rear_leg_right", "rear_feet_right"),
    "front_right":("right", "front_leg_right","front_feet_right"),
}

def leg_fk(side, servo_thigh, servo_knee):
    if side == "left":
        alpha, sigma = servo_thigh - 90.0, servo_knee + 15.0
    else:
        alpha, sigma = 90.0 - servo_thigh, 195.0 - servo_knee
    a = math.radians(alpha)
    g = math.radians(180.0 - sigma - alpha)
    return (-L1_MM*math.sin(a) + L2_MM*math.sin(g),
             L1_MM*math.cos(a) + L2_MM*math.cos(g))

def leg_ik(side, x, z):
    r = math.sqrt(x*x + z*z)
    r = max(abs(L1_MM-L2_MM)+5.0, min(L1_MM+L2_MM-5.0, r))
    r2 = r*r
    sigma = math.degrees(math.acos(max(-1,min(1,(L1_MM*L1_MM+L2_MM*L2_MM-r2)/(2*L1_MM*L2_MM)))))
    delta = math.degrees(math.atan2(x, z))
    tau = math.degrees(math.acos(max(-1,min(1,(L1_MM*L1_MM+r2-L2_MM*L2_MM)/(2*L1_MM*r)))))
    alpha = tau - delta
    if side == "left":
        return 90.0 + alpha, sigma - 15.0
    return 90.0 - alpha, 195.0 - sigma

ANCHOR = {leg: leg_fk(side, STANDING[t], STANDING[k])
          for leg, (side, t, k) in LEG_META.items()}

def do_pawtest():
    """Standing IK validation: each foot slowly traces its swing arc in the air."""
    global state
    state = "PAWTEST"
    log("PAWTEST: each foot traces its swing arc (watch for clean forward arcs)")
    for leg, (side, t_name, k_name) in LEG_META.items():
        log(f"  {leg}...")
        x0, z0 = ANCHOR[leg]
        steps = 60
        for i in range(steps + 1):
            s = i / steps
            dx = -18.0 + 36.0 * s
            dz = -30.0 * math.sin(math.pi * s)
            st, sk = leg_ik(side, x0 + dx, z0 + dz)
            apply({t_name: st, k_name: sk})
            time.sleep(0.05)
        st, sk = leg_ik(side, x0, z0)
        apply({t_name: st, k_name: sk})
        time.sleep(0.5)
        if throttled() & UV:
            fold_to_rest(); return
    state = "STAND"
    log("PAWTEST done — STAND")

CRAWL_STRIDE = 40.0   # mm per cycle
CRAWL_LIFT = 26.0     # mm swing clearance
CRAWL_SHIFT_T = 0.4   # s pre-swing settle
CRAWL_SWING_T = 0.7   # s per leg swing
CRAWL_SWAY = 5.0      # deg lateral lean onto support side
CRAWL_FWD_MM = 12.0   # body shifts forward before REAR-leg swings (their feet sit
                      # far back; lifting one otherwise tips the body backward)
CRAWL_ORDER = ["rear_left", "front_left", "rear_right", "front_right"]

def do_crawl(cycles):
    """8-phase static crawl: shift body, swing one leg, repeat. IK foot paths:
    flat stance lines, clean swing arcs — the proper-legs gait."""
    global state
    state = "CRAWL"
    log(f"CRAWL {cycles} cycles (8-phase, IK)")
    phase_t = CRAWL_SHIFT_T + CRAWL_SWING_T
    cycle_t = 4 * phase_t
    v = CRAWL_STRIDE / cycle_t          # body speed, mm/s
    foot_x = {leg: 0.0 for leg in LEG_META}   # x offset from anchor
    sway_now = 0.0
    bx_now = 0.0                               # body forward shift (mm)
    hist = []
    t_end = time.time() + cycles * cycle_t
    last = time.time()
    while time.time() < t_end:
        loop_start = time.time()
        dt = loop_start - last
        last = loop_start
        if os.path.exists(STOP_FILE): fold_to_rest(); sys.exit(0)
        c = read_cmd()
        if c and c.get("cmd") == "rest":
            break
        tc = (loop_start - (t_end - cycles * cycle_t)) % cycle_t
        idx = int(tc // phase_t)
        pt = tc - idx * phase_t
        swing_leg = CRAWL_ORDER[idx]
        swinging = pt >= CRAWL_SHIFT_T
        # lateral lean: onto RIGHT side (+) while a LEFT leg swings, and vice versa
        # CONTINUOUS body motion — smooth sinusoids, zero lurching:
        # sway peaks mid-left-swings (+), mid-right-swings (-), crosses zero
        # at transitions; fwd shift peaks during each REAR swing (2x rate).
        cyc = tc / cycle_t
        sway_now = 3.0 * math.sin(2 * math.pi * cyc)
        # body BACK during front swings (front feet already sit forward —
        # face-plant risk), forward only for rear swings
        bx_now = 3.0 + 9.0 * math.sin(4 * math.pi * cyc)
        pose = {}
        for leg, (side, t_name, k_name) in LEG_META.items():
            x0, z0 = ANCHOR[leg]
            if leg == swing_leg and swinging:
                s = (pt - CRAWL_SHIFT_T) / CRAWL_SWING_T
                se = 0.5 - 0.5 * math.cos(math.pi * s)
                start_x = foot_x[leg]
                tgt_x = CRAWL_STRIDE / 2.0
                x = start_x + (tgt_x - start_x) * se
                dz = -CRAWL_LIFT * math.sin(math.pi * s)
                if s >= 0.99: foot_x[leg] = tgt_x
                st, sk = leg_ik(side, x0 + x - bx_now, z0 + dz)
            else:
                foot_x[leg] -= v * dt          # stance: flat line backward
                st, sk = leg_ik(side, x0 + foot_x[leg] - bx_now, z0)
            pose[t_name] = st + sway_now       # proven same-sign tilt mechanism
            pose[k_name] = sk
        apply(pose)

        r, p = tilt()
        hist.append((r, p))
        if len(hist) > HZ * 3: hist.pop(0)
        if len(hist) >= HZ:
            mr = sum(h[0] for h in hist) / len(hist)
            mp = sum(h[1] for h in hist) / len(hist)
            if abs(mr) > 15 or abs(mp) > 15:
                log(f"tip guard in crawl: {mr:+.1f}/{mp:+.1f}")
                fold_to_rest(); return
        if int(loop_start * 4) % 4 == 0 and throttled() & UV:
            log("UV in crawl"); fold_to_rest(); return
        time.sleep(max(0, DT - (time.time() - loop_start)))

    ease_to(dict(STANDING, **{n: rest[n] for n in ARM}), seconds=1.2)
    state = "STAND"
    log("CRAWL done — STAND")


def do_dance():
    """Spot-style dance: bounce, sway, rock, then groove (bounce+sway).
    Built from the three proven body primitives; all guards active."""
    global state
    state = "DANCE"
    log("DANCE: let's go")
    KNEE_LIST = list(KNEES)
    FRONT_KNEES = ["front_feet_left", "front_feet_right"]
    REAR_KNEES = ["rear_feet_left", "rear_feet_right"]

    # (name, duration_s, bounce_amp, sway_amp, rock_amp, period_s)
    phases = [
        ("bounce", 8.0, 14.0, 0.0, 0.0, 1.2),
        ("sway",   9.0, 0.0, 8.0, 0.0, 2.6),
        ("rock",   7.0, 0.0, 0.0, 10.0, 2.0),
        ("groove", 8.0, 10.0, 6.0, 0.0, 1.5),
    ]
    hist = []
    for name, dur, b_amp, s_amp, r_amp, period in phases:
        log(f"  dance phase: {name} ({dur:.0f}s)")
        t0 = time.time()
        while True:
            loop_start = time.time()
            t = loop_start - t0
            if t >= dur: break
            if os.path.exists(STOP_FILE): fold_to_rest(); sys.exit(0)
            c = read_cmd()
            if c and c.get("cmd") == "rest":
                fold_to_rest(); return
            env = min(1.0, t / 0.8, (dur - t) / 0.8)   # ramp in/out, no snaps
            w = 2 * math.pi * t / period
            bounce = b_amp * env * math.sin(w)
            sway = s_amp * env * math.sin(w if name != "groove" else w / 2)
            rock = r_amp * env * math.sin(w)

            pose = {}
            for thigh in LEGS:
                pose[thigh] = STANDING[thigh] + sway          # same-sign = roll tilt
            for knee in KNEE_LIST:
                v = STANDING[knee] + KNEES[knee] * bounce     # all fold = body drops
                if rock:
                    v += KNEES[knee] * (rock if knee in FRONT_KNEES else -rock)
                pose[knee] = v
            apply(pose)

            r, p = tilt()
            hist.append((r, p))
            if len(hist) > HZ * 3: hist.pop(0)
            if len(hist) >= HZ:
                mr = sum(h[0] for h in hist) / len(hist)
                mp = sum(h[1] for h in hist) / len(hist)
                if abs(mr) > 15 or abs(mp) > 15:
                    log(f"tip guard in dance: {mr:+.1f}/{mp:+.1f}")
                    fold_to_rest(); return
            if int(t * HZ) % 5 == 0 and throttled() & UV:
                log("UV in dance"); fold_to_rest(); return
            time.sleep(max(0, DT - (time.time() - loop_start)))

    ease_to(dict(STANDING, **{n: rest[n] for n in ARM}), seconds=1.0)
    state = "STAND"
    log("DANCE done — STAND")


def do_walk(cycles, turn=0.0, direction=1):
    """STAND -> WALK N cycles -> STAND. turn in [-1,1]: neg=left, pos=right.
    direction=-1 walks backward (stride phase reversed, same envelope)."""
    global state
    state = "WALK"
    turn = max(-1.0, min(1.0, turn))
    # Turn by BRAKING the inner side only — outer side keeps the proven stride.
    # Stretching the outer side (old approach) drove thighs past their
    # mechanical limits (right-rear hit ~5deg) and toppled the robot.
    inner = "right" if turn > 0 else "left"
    side_scale = {"left": 1.0, "right": 1.0}
    # 0 turn -> 1.0 (normal); 0.5 -> 0 (inner stops); 1.0 -> -1.0 (inner full
    # reverse = pivot in place). Magnitude never exceeds 1.0 -> proven envelope.
    # inner reverse capped at -0.5: full reverse (-1.0) broke diagonal support
    # and toppled the robot (pitch/roll ~26deg). -0.5 pivots tight and stays up.
    side_scale[inner] = max(-0.5, 1.0 - TURN_GAIN * abs(turn))
    if direction < 0:   # backward: reverse both sides' stride phase
        side_scale = {k: -v for k, v in side_scale.items()}
    log(f"WALK {cycles} cycles turn={turn:+.2f} scaleL={side_scale['left']:.2f} scaleR={side_scale['right']:.2f}")
    hist = []
    # tight turns run 25% slower: the loaded inner-rear leg (weakest corner)
    # bounces under scrub load at full pace
    cycle_t = CYCLE_T * (1.25 if abs(turn) > 0.5 else 1.0)
    t0 = time.time()
    total_t = cycles * cycle_t
    i = 0
    while True:
        loop_start = time.time()
        t = loop_start - t0
        if t >= total_t: break
        if os.path.exists(STOP_FILE): fold_to_rest(); sys.exit(0)
        c = read_cmd()
        if c and c.get("cmd") == "rest":   # abort walk on command
            break
        i += 1
        tc = (t % cycle_t) / cycle_t
        half = 0 if tc < 0.5 else 1
        s = (tc * 2.0) % 1.0
        pose = {}
        swing = PAIR_A if half == 0 else PAIR_B
        stance = PAIR_B if half == 0 else PAIR_A
        se = 0.5 - 0.5 * math.cos(math.pi * s)   # eased 0..1: zero velocity at ends
        # rhythmic weight shift: lean away from the swinging diagonal so its
        # feet unload and clear (real-quadruped motion; fixes foot scuffing)
        sway = -WALK_SWAY * math.sin(2 * math.pi * tc)
        def scl(name):
            return side_scale["left" if "left" in name else "right"]
        def fwd_bias(name):
            return THIGHS[name] * -FWD_BIAS   # feet back relative to body = CoM forward
        for thigh in swing:
            # eased swing: gentle liftoff and touchdown, no velocity snap
            pose[thigh] = STANDING[thigh] + sway + fwd_bias(thigh) + THIGHS[thigh] * scl(thigh) * (-SWING_BACK + (SWING_FWD + SWING_BACK) * se)
        for thigh in stance:
            # linear stance: constant backward foot speed = steady push
            pose[thigh] = STANDING[thigh] + sway + fwd_bias(thigh) + THIGHS[thigh] * scl(thigh) * (SWING_FWD - (SWING_FWD + SWING_BACK) * s)
        for knee, leg in LEG_OF_KNEE.items():
            kscale = abs(scl(leg))   # push works in either stride direction
            lift = LIFT + LIFT_BIAS.get(knee, 0.0)
            if leg in swing:
                # symmetric natural lift (the skewed curve read wrong on the rear legs)
                pose[knee] = (STANDING[knee] - KNEES[knee] * PUSH_EXT * kscale * (1 - se)
                              + KNEES[knee] * lift * math.sin(math.pi * s))
            else:
                # stance: extend knee progressively -> foot drives back flat (propulsion)
                pose[knee] = STANDING[knee] - KNEES[knee] * PUSH_EXT * kscale * se
        # subtle arm bob synced to the stride — reads alive instead of robotic
        pose["arm_lift"] = rest["arm_lift"] + ARM_BOB * math.sin(2 * math.pi * tc)
        apply(pose)

        r, p = tilt()
        hist.append((r, p))
        if len(hist) > HZ * 3: hist.pop(0)
        if len(hist) >= HZ:
            mr = sum(h[0] for h in hist) / len(hist)
            mp = sum(h[1] for h in hist) / len(hist)
            if abs(mr) > 15 or abs(mp) > 15:
                log(f"tip guard during walk: {mr:+.1f}/{mp:+.1f}")
                fold_to_rest(); return False
        if i % 5 == 0 and throttled() & UV:
            log("UV during walk"); fold_to_rest(); return False
        if i % (HZ * 2) == 0:
            write_status({"walk_t": round(t, 1), "walk_total": total_t})
        time.sleep(max(0, DT - (time.time() - loop_start)))

    # back to standing pose
    start = dict(cur)
    tgt = dict(STANDING, **{n: rest[n] for n in ARM})
    for step in range(1, 11):
        f = step / 10.0
        apply({n: start[n] + (tgt[n] - start[n]) * f for n in start})
        time.sleep(0.05)
    state = "STAND"
    log("STAND (walk done)")
    return True


def _on_term(signum, frame):
    log("SIGTERM — folding down before exit")
    try: fold_to_rest()
    finally: sys.exit(0)


def main():
    global state
    signal.signal(signal.SIGTERM, _on_term)
    if os.path.exists(STOP_FILE): os.remove(STOP_FILE)
    if os.path.exists(CMD_FILE): os.remove(CMD_FILE)
    # startup safety: previous process may have died holding a pose —
    # release everything so REST is physically true before we assume it
    for ch in range(16): pca.channels[ch].duty_cycle = 0
    log("spot_motiond up — state REST. commands: stand | walk | rest")
    write_status()

    offsets = {n: 0.0 for n in LEGS}
    hist = []
    i = 0
    last_status = 0.0
    while True:
        loop_start = time.time()
        i += 1
        if os.path.exists(STOP_FILE): fold_to_rest(); sys.exit(0)

        c = read_cmd()
        if c:
            cmd = c.get("cmd")
            log(f"command: {c}")
            if cmd == "stand" and state == "REST":
                offsets = {n: 0.0 for n in LEGS}; hist = []
                stand_up()
            elif cmd == "stand" and state == "SIT":
                offsets = {n: 0.0 for n in LEGS}; hist = []
                log("standing up from sit...")
                ease_to(dict(STANDING, **{n: rest[n] for n in ARM}), seconds=2.0)
                state = "STAND"; log("STAND")
            elif cmd == "sit":
                if state == "REST":
                    offsets = {n: 0.0 for n in LEGS}; hist = []
                    if not stand_up():
                        write_status(); continue
                    time.sleep(0.5)
                if state == "STAND":
                    do_sit()
            elif cmd in ("crawl", "pawtest"):
                if state == "REST":
                    offsets = {n: 0.0 for n in LEGS}; hist = []
                    if not stand_up():
                        write_status(); continue
                    time.sleep(1.0)
                if state == "STAND":
                    if cmd == "crawl":
                        do_crawl(int(c.get("cycles", 2)))
                    else:
                        do_pawtest()
                    offsets = {n: 0.0 for n in LEGS}; hist = []
            elif cmd == "dance":
                if state == "REST":
                    offsets = {n: 0.0 for n in LEGS}; hist = []
                    if not stand_up():
                        write_status(); continue
                    time.sleep(1.0)
                if state == "SIT":
                    log("standing up from sit for dance...")
                    ease_to(dict(STANDING, **{n: rest[n] for n in ARM}), seconds=2.0)
                    state = "STAND"; time.sleep(0.5)
                if state == "STAND":
                    do_dance()
                    offsets = {n: 0.0 for n in LEGS}; hist = []
            elif cmd == "wave":
                if state == "REST":
                    offsets = {n: 0.0 for n in LEGS}; hist = []
                    if not stand_up():
                        write_status(); continue
                    time.sleep(0.5)
                if state == "STAND":
                    do_sit(); time.sleep(0.5)
                if state == "SIT":
                    do_wave()
            elif cmd == "walk":
                if state == "REST":
                    offsets = {n: 0.0 for n in LEGS}; hist = []
                    if not stand_up():
                        write_status(); continue
                    time.sleep(1.0)
                if state == "STAND":
                    do_walk(int(c.get("cycles", 8)), float(c.get("turn", 0.0)),
                            int(c.get("dir", 1)))
                    offsets = {n: 0.0 for n in LEGS}; hist = []
            elif cmd == "rest" and state != "REST":
                fold_to_rest()

        if state == "STAND":
            r, p = tilt()
            hist.append((r, p))
            if len(hist) > HZ * 3: hist.pop(0)
            if len(hist) >= HZ:
                mr = sum(h[0] for h in hist) / len(hist)
                mp = sum(h[1] for h in hist) / len(hist)
                if abs(mr) > 15 or abs(mp) > 15:
                    log(f"tip guard in stand: {mr:+.1f}/{mp:+.1f}")
                    fold_to_rest()
            if state == "STAND":
                d_roll = KP * r if abs(r) > DEADBAND else 0.0
                for n in LEGS:
                    stepv = max(-SLEW, min(SLEW, d_roll))
                    offsets[n] = max(-OFFSET_CLAMP, min(OFFSET_CLAMP, offsets[n] + stepv))
                pose = {}
                for n in LEGS:
                    mn, mx = LEG_LIMITS[n]
                    pose[n] = max(mn, min(mx, STANDING[n] + offsets[n]))
                apply(pose)
                if i % 5 == 0 and throttled() & UV:
                    log("UV in stand"); fold_to_rest()
            time.sleep(max(0, DT - (time.time() - loop_start)))
        elif state == "SIT":
            # static grounded pose: tilt guard off (nose-up is expected),
            # power watchdog stays on
            if i % 5 == 0 and throttled() & UV:
                log("UV in sit"); fold_to_rest()
            time.sleep(0.1)
        else:
            time.sleep(0.1)   # REST: idle poll

        if time.time() - last_status > 0.5:
            write_status()
            last_status = time.time()


if __name__ == "__main__":
    try:
        main()
    except SystemExit:
        raise
    except KeyboardInterrupt:
        fold_to_rest()
    except Exception as e:
        log(f"ERROR: {e}")
        try: fold_to_rest()
        finally: raise
