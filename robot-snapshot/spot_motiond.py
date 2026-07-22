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
WAVE_THIGH_UP = 128     # front_leg_right raised way up for the wave
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


_thr_cache = [0]

def _thr_poll():
    while True:
        try:
            out = subprocess.run(["vcgencmd", "get_throttled"], capture_output=True, text=True).stdout
            _thr_cache[0] = int(out.strip().split("=")[1], 16)
        except Exception:
            pass
        time.sleep(1.0)

import threading as _threading
_threading.Thread(target=_thr_poll, daemon=True).start()

def throttled():
    """Cached (1s) — NEVER spawns a subprocess in the motion loop.
    (vcgencmd took 20-50ms; calling it every 5th frame was THE gait stutter.)"""
    return _thr_cache[0]


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


SERVO_FILE = "/tmp/spot_servos.json"
_servo_dump_t = [0.0]

def dump_servos():
    """Publish commanded servo angles (~6Hz max; pure file IO, no I2C)."""
    now = time.time()
    if now - _servo_dump_t[0] < 0.15: return
    _servo_dump_t[0] = now
    try:
        tmp = SERVO_FILE + ".tmp"
        with open(tmp, "w") as f:
            json.dump({"servos": {k: round(v, 1) for k, v in cur.items()},
                       "state": state, "ts": now}, f)
        os.replace(tmp, SERVO_FILE)
    except OSError:
        pass

# per-joint trim: new rear-left knee servo (2026-07-18 swap) sits a few horn-splines off;
# add its offset at the single write point so EVERY state (rest/sit/stand/gait) is corrected.
TRIM = {"rear_feet_left": 12.0}

def apply(pose):
    for n, a in pose.items():
        v = max(0, min(180, a + TRIM.get(n, 0.0)))
        for attempt in range(3):
            try:
                servos[n].angle = v; break
            except OSError:
                if attempt == 2: raise
                time.sleep(0.005)
        cur[n] = v
    dump_servos()


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
    log("standing up (power-gentle)...")
    for n in servos:
        apply({n: rest[n]}); time.sleep(0.15)   # slow stagger: inrush spread out
        if throttled() & UV: fold_to_rest(); return False
    time.sleep(0.5)
    start = dict(cur)
    target = dict(STANDING, **{n: rest[n] for n in ARM})
    for step in range(1, 27):
        f = step / 26.0
        apply({n: start[n] + (target[n] - start[n]) * f for n in target})
        if throttled() & UV:
            time.sleep(1.5)
            if throttled() & UV: fold_to_rest(); return False
        time.sleep(0.07)
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
    """From SIT: raise front-right paw high and wag it in a smooth sinusoid
    with a live thigh bob — fluid, not robotic A-B."""
    global state
    log("WAVE: raising front-right paw...")
    ease_to({"front_leg_right": WAVE_THIGH_UP, "front_feet_right": WAVE_KNEE_A}, seconds=0.8)
    mid = (WAVE_KNEE_A + WAVE_KNEE_B) / 2.0
    span = (WAVE_KNEE_B - WAVE_KNEE_A) / 2.0
    t = 0.0
    last = time.time()
    while t < 3 * 0.8:            # three smooth wags
        loop_start = time.time()
        dt = loop_start - last
        last = loop_start
        t += dt
        env = min(1.0, t / 0.3, (2.4 - t) / 0.3)
        wv = 2 * math.pi * t / 0.8
        apply({"front_feet_right": mid + span * env * math.sin(wv),
               "front_leg_right": WAVE_THIGH_UP + 5.0 * env * math.sin(wv * 0.5)})
        time.sleep(max(0, DT - (time.time() - loop_start)))
    log("WAVE: paw back down")
    ease_to({"front_leg_right": SIT_POSE["front_leg_right"],
             "front_feet_right": SIT_POSE["front_feet_right"]}, seconds=0.8)
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

# Calibration poses for photo-based joint measurement (robot ON A STAND,
# feet free). Shoulders at calibrated straight values -> leg planes square
# to a side-on camera. P2 = the standing pose itself, tying the fit to our anchors.
CAL_POSES = [
    {"tL": 90,  "tR": 90, "kL": 90,  "kR": 90},
    {"tL": 120, "tR": 60, "kL": 60,  "kR": 120},
    {"tL": 150, "tR": 30, "kL": 118, "kR": 82},
]

def do_calpose(n):
    """Hold calibration pose n for photographing. ROBOT MUST BE ON A STAND."""
    global state
    p = CAL_POSES[max(0, min(len(CAL_POSES) - 1, n))]
    log(f"CALPOSE {n}: thighs L{p['tL']}/R{p['tR']} knees L{p['kL']}/R{p['kR']} (hold for photos)")
    target = {
        "rear_leg_left": p["tL"], "front_leg_left": p["tL"],
        "rear_leg_right": p["tR"], "front_leg_right": p["tR"],
        "rear_feet_left": p["kL"], "front_feet_left": p["kL"],
        "rear_feet_right": p["kR"], "front_feet_right": p["kR"],
        "rear_shoulder_left": STANDING["rear_shoulder_left"],
        "rear_shoulder_right": STANDING["rear_shoulder_right"],
        "front_shoulder_left": STANDING["front_shoulder_left"],
        "front_shoulder_right": STANDING["front_shoulder_right"],
    }
    target.update({a: rest[a] for a in ARM})
    if not cur:   # from REST: energize gently at current target of rest first
        for name in servos:
            apply({name: rest[name]}); time.sleep(0.04)
        time.sleep(0.3)
    ease_to(target, seconds=2.0)
    state = "CALPOSE"
    log(f"CALPOSE {n} holding — photograph both sides, square-on")


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

CRAWL_STRIDE = 60.0   # mm per cycle
CRAWL_LIFT = 26.0     # mm swing clearance
CRAWL_SHIFT_T = 0.4   # s all-feet-planted transition window at each phase start
CRAWL_SWING_T = 0.5   # s per leg swing
CRAWL_ABDUCT = 10.0   # deg hip abduction: real lateral CoM shift (~33mm)
CRAWL_BX_REAR = 20.0  # mm body-forward plateau during rear swings
CRAWL_BX_FRONT = -8.0   # mm body-back during front swings (was -18: overloaded the rear)
CRAWL_CROUCH = 18.0     # mm lower body during crawl: shorter lever = less servo sag
CRAWL_KR = 1.2          # mm per deg roll  — IMU posture regulator gains
CRAWL_KP = 1.2          # mm per deg pitch (rear servos SAG under sustained load;
                        # closed-loop leg length holds the body level)
# SIM-VALIDATED in the CALIBRATED stance (no posture change): margin +25.6mm
# nominal, ~+17mm with CoM off 10mm — the dog keeps his real stance.
ABDUCT_OUT = {"rear_left": +1, "rear_right": -1, "front_left": -1, "front_right": +1}
SHOULDER_OF_LEG = {"rear_left": "rear_shoulder_left", "rear_right": "rear_shoulder_right",
                   "front_left": "front_shoulder_left", "front_right": "front_shoulder_right"}
CRAWL_ORDER = ["rear_left", "front_left", "rear_right", "front_right"]

def _smoothstep(u):
    u = max(0.0, min(1.0, u))
    return u*u*(3-2*u)

def do_crawl(cycles):
    """SIM-VALIDATED 8-phase crawl: symmetric stance, plateau body shifts
    (lateral via hip abduction, fore-aft via IK), transitions only while all
    four feet are planted. One leg swings at a time."""
    global state
    state = "CRAWL"
    log(f"CRAWL {cycles} cycles (calibrated stance, abduct {CRAWL_ABDUCT}, bx +{CRAWL_BX_REAR}/{CRAWL_BX_FRONT})")
    # ease down into the crawl crouch (all feet planted)
    for i in range(1, 21):
        f = _smoothstep(i / 20)
        pose = {}
        for leg, (side, t_name, k_name) in LEG_META.items():
            xa, z0 = ANCHOR[leg]
            st, sk = leg_ik(side, xa, z0 - CRAWL_CROUCH * f)
            pose[t_name] = st; pose[k_name] = sk
        apply(pose)
        time.sleep(0.05)

    phase_t = CRAWL_SHIFT_T + CRAWL_SWING_T
    cycle_t = 4 * phase_t
    v = CRAWL_STRIDE / cycle_t
    foot_x = {leg: 0.0 for leg in LEG_META}
    hist = []
    t0 = time.time()
    t_end = t0 + cycles * cycle_t
    last = t0
    # plateau targets per phase: body RIGHT (-) while LEFT legs swing (ph 0,1)
    body_left_of = [-CRAWL_ABDUCT, -CRAWL_ABDUCT, +CRAWL_ABDUCT, +CRAWL_ABDUCT]
    bx_of = [CRAWL_BX_REAR, CRAWL_BX_FRONT, CRAWL_BX_REAR, CRAWL_BX_FRONT]
    last_logged_phase = -1
    zc = {leg: 0.0 for leg in LEG_META}   # posture-regulator z corrections (mm)
    while time.time() < t_end:
        loop_start = time.time()
        dt = loop_start - last
        last = loop_start
        if os.path.exists(STOP_FILE): fold_to_rest(); sys.exit(0)
        c = read_cmd()
        if c:
            if c.get("cmd") == "stop": break            # -> eased return to STAND
            if c.get("cmd") == "rest": fold_to_rest(); return
        tc = (loop_start - t0) % cycle_t
        idx = int(tc // phase_t)
        pt = tc - idx * phase_t
        swing_leg = CRAWL_ORDER[idx]
        swinging = pt >= CRAWL_SHIFT_T
        u = _smoothstep(pt / CRAWL_SHIFT_T) if pt < CRAWL_SHIFT_T else 1.0
        body_left = body_left_of[(idx-1) % 4] + (body_left_of[idx] - body_left_of[(idx-1) % 4]) * u
        bx = bx_of[(idx-1) % 4] + (bx_of[idx] - bx_of[(idx-1) % 4]) * u

        # per-phase telemetry: attitude + commanded body state at each swing start
        if swinging and idx != last_logged_phase:
            r_t, p_t = tilt()
            log(f"  ph{idx} swing={swing_leg:<11} roll={r_t:+5.1f} pitch={p_t:+5.1f} "
                f"bodyL={body_left:+5.1f}deg bx={bx:+5.1f}mm")
            last_logged_phase = idx
        pose = {}
        for leg, (side, t_name, k_name) in LEG_META.items():
            xa, z0 = ANCHOR[leg]
            zleg = z0 - CRAWL_CROUCH + zc[leg]
            if leg == swing_leg and swinging:
                s = (pt - CRAWL_SHIFT_T) / CRAWL_SWING_T
                se = 0.5 - 0.5 * math.cos(math.pi * s)
                x = foot_x[leg] + (CRAWL_STRIDE / 2.0 - foot_x[leg]) * se
                dz = -CRAWL_LIFT * math.sin(math.pi * s)
                if s >= 0.99: foot_x[leg] = CRAWL_STRIDE / 2.0
                st, sk = leg_ik(side, xa + x - bx, zleg + dz)
            else:
                foot_x[leg] -= v * dt
                st, sk = leg_ik(side, xa + foot_x[leg] - bx, zleg)
            pose[t_name] = st
            pose[k_name] = sk
            side_sign = 1 if side == "right" else -1
            sh = SHOULDER_OF_LEG[leg]
            pose[sh] = STANDING[sh] + ABDUCT_OUT[leg] * side_sign * body_left
        apply(pose)

        r, p = tilt()
        # IMU POSTURE REGULATOR: servos sag under single-leg load — measure the
        # body attitude and extend/shorten legs to hold level. roll+ = left low
        # -> extend left legs; pitch+ = rear high -> extend front / shorten rear.
        for leg in LEG_META:
            want = (CRAWL_KR * r * (1 if "left" in leg else -1)
                    + CRAWL_KP * p * (1 if "front" in leg else -1))
            stepz = max(-2.0, min(2.0, want - zc[leg]))
            zc[leg] = max(-22.0, min(22.0, zc[leg] + stepz))
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

    # return: ease straight back to the standing pose (we never left the stance)
    ease_to(dict(STANDING, **{n: rest[n] for n in ARM}), seconds=1.0)
    state = "STAND"
    log("CRAWL done — STAND")

HOP_CROUCH_KNEE, HOP_CROUCH_THIGH = 46.0, 26.0  # MAX preload fold (deg)
HOP_EXT_KNEE, HOP_EXT_THIGH = 30.0, 18.0         # 90% launch: explosive but land-able

def do_hop():
    """Max four-leg hop (best of 9 iterations): deep coil, full-stroke launch
    with rear-first stagger (stays level), soft-crouch catch. This is the
    hardest vertical this hardware produces at 5V."""
    global state
    state = "HOP"
    log("HOP: coil...")
    fold = {}
    for knee in KNEES:
        fold[knee] = STANDING[knee] + KNEES[knee] * HOP_CROUCH_KNEE
    for thigh in THIGHS:
        fold[thigh] = STANDING[thigh] + (-THIGHS[thigh]) * HOP_CROUCH_THIGH
    ease_to(fold, seconds=0.45)
    time.sleep(0.2)

    log("HOP: launch!")
    ext_rear, ext_front = {}, {}
    for knee in KNEES:
        d = ext_rear if "rear" in knee else ext_front
        scale = 1.0 if "rear" in knee else 0.85
        d[knee] = STANDING[knee] - KNEES[knee] * HOP_EXT_KNEE * scale
    for thigh in THIGHS:
        d = ext_rear if "rear" in thigh else ext_front
        scale = 1.0 if "rear" in thigh else 0.85
        d[thigh] = STANDING[thigh] - (-THIGHS[thigh]) * HOP_EXT_THIGH * scale
    apply(ext_rear)
    time.sleep(0.06)
    apply(ext_front)
    time.sleep(0.30)              # full stroke completes

    # soft catch
    catch = {}
    for knee in KNEES:
        catch[knee] = STANDING[knee] + KNEES[knee] * (HOP_CROUCH_KNEE * 0.6)
    for thigh in THIGHS:
        catch[thigh] = STANDING[thigh] + (-THIGHS[thigh]) * (HOP_CROUCH_THIGH * 0.6)
    apply(catch)
    time.sleep(0.25)
    # active recovery in the crouch: level out BEFORE rising (landing on
    # carpet is stochastic — earlier runs stuck at +/-2, this one hit -20)
    for _ in range(12):           # up to 0.6s of roll correction
        r, p = tilt()
        if abs(r) < 4:
            break
        adj = max(-3.0, min(3.0, 0.3 * r))
        for thigh in THIGHS:
            catch[thigh] = catch[thigh] + adj
        apply({t: catch[t] for t in THIGHS})
        time.sleep(0.05)
    ease_to(dict(STANDING), seconds=0.6)
    time.sleep(0.4)

    r, p = tilt()
    t = throttled()
    log(f"HOP: landed roll={r:+.1f} pitch={p:+.1f} thr=0x{t:X}")
    if abs(r) > 15 or abs(p) > 15:
        fold_to_rest(); return
    if t & UV:
        log("UV after hop"); fold_to_rest(); return
    state = "STAND"

LIMP_ORDER = ["front_left", "rear_left", "front_right"]   # rear_right = peg
LIMP_STRIDE_F, LIMP_STRIDE_B, LIMP_LIFT = 9.0, 11.0, 22.0
LIMP_LEAN = -3.5      # same-sign thigh bias: weight onto the LEFT, off the bad corner

def do_limp(cycles):
    """Injured-leg gait: rear-right held rigid as a peg (static load only),
    weight biased away from it, the three good legs walk one at a time and
    the lame paw drags on its rounded foot. It's a limp — and it moves."""
    global state
    state = "WALK"
    log(f"LIMP {cycles} cycles (rear-right pegged)")
    phase_t = 0.75                 # per swing phase (3 per cycle)
    cycle_t = 3 * phase_t
    v = (LIMP_STRIDE_F + LIMP_STRIDE_B) / cycle_t / 2.5   # stance creep rate deg/s
    hist = []
    foot = {leg: 0.0 for leg in LIMP_ORDER}   # thigh offset from bias point
    t0 = time.time()
    total_t = cycles * cycle_t
    last = t0
    i = 0
    while True:
        loop_start = time.time()
        t = loop_start - t0
        if t >= total_t: break
        if os.path.exists(STOP_FILE): fold_to_rest(); sys.exit(0)
        c = read_cmd()
        if c:
            if c.get("cmd") == "stop": break
            if c.get("cmd") == "rest": fold_to_rest(); return
        i += 1
        dt = loop_start - last
        last = loop_start
        tc = (t % cycle_t)
        idx = int(tc // phase_t)
        s = (tc - idx * phase_t) / phase_t
        se = 0.5 - 0.5 * math.cos(math.pi * s)
        swing_leg = LIMP_ORDER[idx]

        pose = {}
        # peg leg: rigid at standing (plus lean), zero dynamics
        pose["rear_leg_right"] = STANDING["rear_leg_right"] + LIMP_LEAN
        pose["rear_feet_right"] = STANDING["rear_feet_right"]
        for leg in LIMP_ORDER:
            t_name = leg.replace("front_", "front_leg_").replace("rear_", "rear_leg_") if "_leg_" not in leg else leg
            t_name = {"front_left": "front_leg_left", "rear_left": "rear_leg_left",
                      "front_right": "front_leg_right"}[leg]
            k_name = {"front_left": "front_feet_left", "rear_left": "rear_feet_left",
                      "front_right": "front_feet_right"}[leg]
            if leg == swing_leg:
                foot[leg] = -LIMP_STRIDE_B + (LIMP_STRIDE_F + LIMP_STRIDE_B) * se
                lift = LIMP_LIFT * math.sin(math.pi * s)
                pose[k_name] = STANDING[k_name] + KNEES[k_name] * lift
            else:
                foot[leg] -= v * dt
                foot[leg] = max(-LIMP_STRIDE_B, foot[leg])
                pose[k_name] = STANDING[k_name]
            pose[t_name] = STANDING[t_name] + LIMP_LEAN + THIGHS[t_name] * foot[leg]
        apply(pose)

        if i % 3 == 0:
            r, p = tilt()
            hist.append((r, p))
            if len(hist) > HZ: hist.pop(0)
            if len(hist) >= HZ // 3:
                mr = sum(h[0] for h in hist) / len(hist)
                mp = sum(h[1] for h in hist) / len(hist)
                if abs(mr) > 18 or abs(mp) > 18:
                    log(f"tip guard in limp: {mr:+.1f}/{mp:+.1f}")
                    fold_to_rest(); return
        if throttled() & UV:
            fold_to_rest(); return
        time.sleep(max(0, DT - (time.time() - loop_start)))

    ease_to(dict(STANDING, **{n: rest[n] for n in ARM}), seconds=0.8)
    state = "STAND"
    log("LIMP done — STAND")

def do_dance(move="full"):
    """Continuous-flow dance: phases CROSSFADE (no dead stops), shoulders twist,
    single phase accumulator keeps the rhythm unbroken. move: full|bounce|sway|rock|groove."""
    global state
    state = "DANCE"
    log(f"DANCE: {move} (fluid)")
    # (name, dur, bounce, sway, rock, twist, period)
    phases = [
        ("bounce", 7.0, 18.0, 0.0, 0.0, 0.0, 1.1),
        ("sway",   8.0, 0.0, 11.0, 0.0, 6.0, 2.4),
        ("rock",   7.0, 0.0, 0.0, 13.0, 0.0, 1.8),
        ("groove", 8.0, 12.0, 8.0, 0.0, 8.0, 1.4),
    ]
    if move != "full":
        phases = [p for p in phases if p[0] == move] * 2
    FRONT_KNEES = ("front_feet_left", "front_feet_right")
    BLEND = 0.7
    total = sum(p[1] for p in phases)
    hist = []
    w = 0.0                      # continuous phase accumulator — rhythm never resets
    t = 0.0
    last = time.time()
    boundaries = []
    acc = 0.0
    for p in phases:
        boundaries.append((acc, acc + p[1], p))
        acc += p[1]

    while t < total:
        loop_start = time.time()
        dt = loop_start - last
        last = loop_start
        t += dt
        if t >= total: break
        if os.path.exists(STOP_FILE): fold_to_rest(); sys.exit(0)
        c = read_cmd()
        if c and c.get("cmd") == "stop": break
        if c and c.get("cmd") == "rest": fold_to_rest(); return

        # current phase + crossfade from previous
        idx = next(i for i, (a, b, _) in enumerate(boundaries) if a <= t < b)
        a, b, P = boundaries[idx]
        lt = t - a
        if idx > 0 and lt < BLEND:
            u = 0.5 - 0.5 * math.cos(math.pi * lt / BLEND)
            Q = boundaries[idx - 1][2]
            _, _, bb, ss, rr, tw, per = [None, None] + [Q[i] + (P[i] - Q[i]) * u for i in range(2, 7)]
        else:
            bb, ss, rr, tw, per = P[2], P[3], P[4], P[5], P[6]
        env = min(1.0, t / 0.8, (total - t) / 0.8)   # ONLY global in/out ramps
        w += 2 * math.pi * dt / per
        bounce = bb * env * math.sin(w)
        sway = ss * env * math.sin(w)
        rock = rr * env * math.sin(w)
        twist = tw * env * math.sin(w * 0.5)          # lazy half-time hips

        pose = {}
        for thigh in LEGS:
            pose[thigh] = STANDING[thigh] + sway
        for knee in KNEES:
            v = STANDING[knee] + KNEES[knee] * bounce
            if rr:
                v += KNEES[knee] * (rock if knee in FRONT_KNEES else -rock)
            pose[knee] = v
        if tw:
            for leg, sh in SHOULDER_OF_LEG.items():
                pose[sh] = STANDING[sh] + twist       # same-sign = pure yaw twist
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

    # shoulders home + settle
    apply({sh: STANDING[sh] for sh in SHOULDER_OF_LEG.values()})
    ease_to(dict(STANDING, **{n: rest[n] for n in ARM}), seconds=0.8)
    state = "STAND"
    log("DANCE done — STAND")

def do_walk(cycles, turn=0.0, direction=1):
    """CPG trot — every joint is a continuous phase-shifted sinusoid (no
    piecewise swing/stance, no velocity jumps anywhere = maximal fluidity).
    Thigh: A*sin(th). Knee: lift*relu(cos th)^2 while sweeping forward,
    push*relu(-cos th)^2 while sweeping back (both C1-smooth half-waves).
    Diagonal pairs run 180deg out of phase. turn: differential A + shoulder yaw."""
    global state
    state = "WALK"
    turn = max(-1.0, min(1.0, turn))
    inner = "right" if turn > 0 else "left"
    side_scale = {"left": 1.0, "right": 1.0}
    side_scale[inner] = max(-0.6, 1.0 - TURN_GAIN * abs(turn))
    if direction < 0:
        side_scale = {k: -v for k, v in side_scale.items()}
    def scl(name):
        v = side_scale["left" if "left" in name else "right"]
        if "rear" in name and abs(turn) > 0.1:
            v = max(-0.3, v)      # inner rear stays supportive in turns
        return v
    # GAIT anchors: the approved walk was tuned on rear_left 150/118 — the
    # squared *standing* pose (162/124) runs the left swing into the 180deg
    # servo limit (162+17=179 clamps; right rear then looks like it kicks back)
    GAIT = dict(STANDING)
    GAIT["rear_leg_left"] = 150.0
    GAIT["rear_feet_left"] = 118.0
    A = (SWING_FWD + SWING_BACK) / 2.0          # sine amplitude (deg)
    cycle_t = CYCLE_T * (1.25 if abs(turn) > 0.5 else 1.0)
    log(f"WALK {cycles} cycles turn={turn:+.2f} dir={direction} (CPG fluid)")
    hist = []
    t0 = time.time()
    total_t = cycles * cycle_t
    last = t0
    i = 0
    while True:
        loop_start = time.time()
        t = loop_start - t0
        if t >= total_t: break
        if os.path.exists(STOP_FILE): fold_to_rest(); sys.exit(0)
        c = read_cmd()
        if c:
            if c.get("cmd") == "stop": break
            if c.get("cmd") == "rest": fold_to_rest(); return False
        i += 1
        tc = (t % cycle_t) / cycle_t
        half = 0 if tc < 0.5 else 1
        s = (tc * 2.0) % 1.0
        swing = PAIR_A if half == 0 else PAIR_B
        stance = PAIR_B if half == 0 else PAIR_A
        se = 0.5 - 0.5 * math.cos(math.pi * s)
        sway = -WALK_SWAY * math.sin(2 * math.pi * tc)
        pose = {}
        for thigh in swing:
            pose[thigh] = GAIT[thigh] + sway + THIGHS[thigh] * scl(thigh) * (-SWING_BACK + (SWING_FWD + SWING_BACK) * se)
        for thigh in stance:
            pose[thigh] = GAIT[thigh] + sway + THIGHS[thigh] * scl(thigh) * (SWING_FWD - (SWING_FWD + SWING_BACK) * s)
        for knee, leg in LEG_OF_KNEE.items():
            kscale = abs(scl(leg))
            lift = LIFT + LIFT_BIAS.get(knee, 0.0)
            if leg in swing:
                pose[knee] = (GAIT[knee] - KNEES[knee] * PUSH_EXT * kscale * (1 - se)
                              + KNEES[knee] * lift * math.sin(math.pi * s))
            else:
                pose[knee] = GAIT[knee] - KNEES[knee] * PUSH_EXT * kscale * se
        if abs(turn) > 0.1:
            yaw_dir = 1.0 if turn < 0 else -1.0
            amp = 12.0 * abs(turn)
            for thigh in stance:
                sh = SHOULDER_OF_LEG[thigh.replace("_leg", "")]
                pose[sh] = STANDING[sh] + yaw_dir * amp * (s - 0.5)
            for thigh in swing:
                sh = SHOULDER_OF_LEG[thigh.replace("_leg", "")]
                pose[sh] = STANDING[sh] + yaw_dir * amp * (0.5 - se)
        pose["arm_lift"] = rest["arm_lift"] + ARM_BOB * math.sin(2 * math.pi * tc)
        apply(pose)

        if i % 2 == 0:                      # guard sampling off the hot path
            r, p = tilt()
            hist.append((r, p))
            if len(hist) > 30: hist.pop(0)      # ~3s window at 10Hz sampling
            if len(hist) >= 10:
                mr = sum(h[0] for h in hist) / len(hist)
                mp = sum(h[1] for h in hist) / len(hist)
                if abs(mr) > 20 or abs(mp) > 20:
                    log(f"tip guard during walk: {mr:+.1f}/{mp:+.1f}")
                    fold_to_rest(); return False
        if throttled() & UV:                 # cached — free
            fold_to_rest(); return False
        if i % (HZ * 2) == 0:
            write_status({"walk_t": round(t, 1), "walk_total": total_t})
        time.sleep(max(0, DT - (time.time() - loop_start)))

    start = dict(cur)
    tgt = dict(STANDING, **{n: rest[n] for n in ARM})
    for step in range(1, 11):
        f = step / 10.0
        apply({n: start[n] + (tgt[n] - start[n]) * f for n in start if n in tgt})
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
            elif cmd == "calpose":
                do_calpose(int(c.get("n", 0)))
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
            elif cmd == "limp":
                if state == "REST":
                    offsets = {n: 0.0 for n in LEGS}; hist = []
                    if not stand_up():
                        write_status(); continue
                    time.sleep(1.0)
                if state == "STAND":
                    do_limp(int(c.get("cycles", 6)))
                    offsets = {n: 0.0 for n in LEGS}; hist = []
            elif cmd == "hop":
                if state == "REST":
                    offsets = {n: 0.0 for n in LEGS}; hist = []
                    if not stand_up():
                        write_status(); continue
                    time.sleep(0.8)
                if state == "STAND":
                    do_hop()
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
                    do_dance(str(c.get("move", "full")))
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
            elif cmd == "stop":
                if state in ("SIT", "CALPOSE"):
                    log("stop: easing back to stand")
                    ease_to(dict(STANDING, **{n: rest[n] for n in ARM}), seconds=2.0)
                    state = "STAND"
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
        elif state == "CALPOSE":
            time.sleep(0.1)   # hold pose; UV irrelevant on a stand w/ legs free
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
