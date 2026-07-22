#!/usr/bin/env python3
"""SpotMicro persistent standing controller.
Stands up (micro-stepped pop), then continuously self-corrects stance at 20Hz.
Every exit path folds gently back to crouch before releasing (no flop-drops).
Stop with: touch /tmp/spot_stop
Log: ~/stand_hold.log (this file's stdout)
"""
import json, time, subprocess, sys, math, os, statistics

import busio
from board import SCL, SDA
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo as af_servo
from mpu6050 import mpu6050

STANDING = {
    # rear extended 8deg fuller than runtime defaults (user: "didn't stand fully
    # with his back legs"); fold direction is left->180/right->0, so fuller
    # stand = left lower / right higher, feet follow
    # rear-left is this robot's chronically low corner (history: 140->135 in the
    # runtime "to help level"); extend it 6deg beyond the mirrored value
    "rear_leg_left": 150, "rear_leg_right": 30,
    "front_leg_left": 130, "front_leg_right": 50,
    "rear_feet_left": 118, "rear_feet_right": 82,
    "front_feet_left": 90, "front_feet_right": 120,
    "rear_shoulder_left": 90, "rear_shoulder_right": 90,
    "front_shoulder_left": 108, "front_shoulder_right": 108,
}
ARM = ["arm_rotation", "arm_lift", "arm_range", "arm_cam_tilt"]
LEGS = ["rear_leg_left", "front_leg_left", "rear_leg_right", "front_leg_right"]

UV = 0x1
HZ = 20
DT = 1.0 / HZ
KP = 0.15            # half the runtime's gain
DEADBAND = 4.0       # deg — only react to real lean, not carpet noise (windup fix)
SLEW = 0.9           # max deg change per cycle per leg
OFFSET_CLAMP = 8.0   # capped small: corrections handle disturbances, never distort the pose
TIP = 20.0           # sustained deviation from standing attitude -> fold down
STOP_FILE = "/tmp/spot_stop"

cfg = json.load(open(os.path.expanduser("~/spotmicroai.json")))
sc = {}
for g in cfg["motion_controller"]:
    if "servos" in g:
        for sd in g["servos"]:
            for n, a in sd.items():
                sc[n] = a[0]

# NOTE: the front-left shoulder/thigh channel cross is now fixed in
# ~/spotmicroai.json itself (2026-07-05) — no swap needed here.

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
# Shoulders NEVER rotate during stand/fold — leg planes stay straight through
# every phase; only thighs and knees fold. Rest = standing values.
# (front_shoulder_left horn is ~22deg off-center, hence 130 vs the right's 108)
rest["front_shoulder_left"] = 108
rest["front_shoulder_right"] = 108
cur = {}  # current commanded angles


def log(msg):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


def throttled():
    out = subprocess.run(["vcgencmd", "get_throttled"], capture_output=True, text=True).stdout
    return int(out.strip().split("=")[1], 16)


def tilt():
    """Median-of-5 accel tilt — rejects dynamic spikes. I2C-retry armored."""
    rs, ps = [], []
    for _ in range(5):
        for attempt in range(3):
            try:
                a = mpu.get_accel_data()
                break
            except OSError:
                if attempt == 2:
                    raise
                time.sleep(0.005)
        rs.append(math.atan2(a["y"], a["z"]) * 57.2958)
        ps.append(math.atan2(-a["x"], math.sqrt(a["y"]**2 + a["z"]**2)) * 57.2958)
    return statistics.median(rs), statistics.median(ps)


def apply(pose):
    for n, a in pose.items():
        v = max(0, min(180, a))
        for attempt in range(3):
            try:
                servos[n].angle = v
                break
            except OSError:
                if attempt == 2:
                    raise
                time.sleep(0.005)
        cur[n] = v


def fold_to_crouch_and_release(reason):
    log(f"FOLDING DOWN: {reason}")
    start = dict(cur)
    for step in range(1, 16):
        f = step / 15.0
        pose = {n: start[n] + (rest[n] - start[n]) * f for n in start}
        apply(pose)
        time.sleep(0.06)
    time.sleep(0.3)
    for ch in range(16):
        pca.channels[ch].duty_cycle = 0
    log("crouched + released. exiting.")


def main():
    if os.path.exists(STOP_FILE):
        os.remove(STOP_FILE)

    if throttled() & UV:
        log("UV before start — aborting without moving"); sys.exit(2)

    r_rest, p_rest = tilt()
    log(f"rest attitude: roll={r_rest:.2f} pitch={p_rest:.2f} throttled=0x{throttled():X}")

    log("Phase 1: energize all 16 servos at rest (staggered)...")
    for n in servos:
        servos[n].angle = rest[n]
        cur[n] = rest[n]
        time.sleep(0.05)
        t = throttled()
        log(f"  energized {n:<22} ch{sc[n]['channel']:>2} @ {rest[n]:>3}deg  thr=0x{t:X}")
        if t & UV:
            fold_to_crouch_and_release("UV during energize"); sys.exit(3)
    time.sleep(0.3)

    log("Phase 2: micro-stepped pop to standing (arm held at rest)...")
    start = dict(cur)
    target = dict(STANDING, **{n: rest[n] for n in ARM})
    POP = 18  # gentler than 10 — rear takes the weight shift gradually
    for step in range(1, POP + 1):
        f = step / POP
        apply({n: start[n] + (target[n] - start[n]) * f for n in target})
        r, p = tilt()
        t = throttled()
        log(f"  slice {step:>2}/{POP} f={f:.1f} roll={r:+.1f} pitch={p:+.1f} thr=0x{t:X}")
        if t & UV:
            log("  UV during pop — freezing 1.5s")
            time.sleep(1.5)
            if throttled() & UV:
                fold_to_crouch_and_release("persistent UV during pop"); sys.exit(3)
        time.sleep(0.08)

    log("settling 1.0s...")
    time.sleep(1.0)
    # Reference = TRUE LEVEL (0,0). The IMU mount is true on this robot
    # (verified flat readings ~1.3/0.3 deg). Never measure "flat" at launch —
    # a just-folded or hand-held robot poisons the reference (caused a false
    # 25deg "tip" abort). The rest reading logged above is placement info only.
    r0, p0 = 0.0, 0.0
    r_now, p_now = tilt()
    log(f"reference=LEVEL(0,0) | post-pop attitude roll={r_now:+.1f} pitch={p_now:+.1f} = initial error")

    log(f"Phase 3: continuous self-correction at {HZ}Hz. stop with: touch {STOP_FILE}")
    offsets = {n: 0.0 for n in LEGS}
    hist = []
    tip_strikes = 0
    uv_strikes = 0
    steady = {"roll": 0, "pitch": 0}
    last = {"roll": 0.0, "pitch": 0.0}
    i = 0
    while True:
        loop_start = time.time()
        i += 1

        if os.path.exists(STOP_FILE):
            fold_to_crouch_and_release("stop file"); sys.exit(0)

        r, p = tilt()
        re, pe = r - r0, p - p0

        # tip guard: judged on the ROLLING MEAN over one sway period (3s) —
        # rhythmic motion reads as fake tilt on an accelerometer (dynamic
        # acceleration); real tipping persists in the mean, sway cancels out.
        hist.append((re, pe))
        if len(hist) > HZ * 3:
            hist.pop(0)
        if len(hist) >= HZ:
            mr = sum(h[0] for h in hist) / len(hist)
            mp = sum(h[1] for h in hist) / len(hist)
            if abs(mr) > 15 or abs(mp) > 15:
                fold_to_crouch_and_release(f"sustained tip (3s mean): droll={mr:+.1f} dpitch={mp:+.1f}"); sys.exit(4)

        # power watchdog every 5th cycle (~4Hz)
        if i % 5 == 0:
            if throttled() & UV:
                uv_strikes += 1
                log(f"UV strike {uv_strikes}")
                if uv_strikes >= 3:
                    fold_to_crouch_and_release("repeated under-voltage"); sys.exit(3)
            else:
                uv_strikes = max(0, uv_strikes - 1)

        # NOTE: no adaptive rebase — it absorbed real lean as "new normal"
        # (observed 3x). Flat is flat; the reference never moves.

        # ROLL-ONLY correction. Pitch correction is disabled: left/right leg
        # servos are mirrored (left extends as angle drops, right as it rises),
        # so a same-sign pitch offset TWISTS the front pair instead of lowering
        # it — observed lifting the front-right foot off the floor. Pitch is
        # handled statically by the rear-extended STANDING pose.
        # Roll sign verified empirically: POSITIVE offsets reduce positive roll.
        d_roll = KP * re if abs(re) > DEADBAND else 0.0

        # BALANCE ONLY — no choreography. Corrections exist solely to hold level.
        want = {
            "rear_leg_left":  d_roll,
            "front_leg_left": d_roll,
            "rear_leg_right": d_roll,
            "front_leg_right": d_roll,
        }
        for n in LEGS:
            step_amt = max(-SLEW, min(SLEW, want[n]))
            offsets[n] = max(-OFFSET_CLAMP, min(OFFSET_CLAMP, offsets[n] + step_amt))

        lo = {"rear_leg_left": (80, 170), "front_leg_left": (90, 170),
              "rear_leg_right": (10, 100), "front_leg_right": (20, 90)}
        pose = {}
        for n in LEGS:
            mn, mx = lo[n]
            pose[n] = max(mn, min(mx, STANDING[n] + offsets[n]))
        apply(pose)

        correcting = any(abs(want[n]) > 0 for n in LEGS)
        if i % HZ == 0 or (correcting and i % 5 == 0):  # 1s telemetry + 4Hz while correcting
            log(f"t={i/HZ:6.1f}s droll={re:+5.1f} dpitch={pe:+5.1f} "
                f"corr={'Y' if correcting else 'n'} "
                f"off={{'RL':{offsets['rear_leg_left']:+.1f},'FL':{offsets['front_leg_left']:+.1f},"
                f"'RR':{offsets['rear_leg_right']:+.1f},'FR':{offsets['front_leg_right']:+.1f}}} "
                f"legs={{'RL':{pose['rear_leg_left']:.0f},'FL':{pose['front_leg_left']:.0f},"
                f"'RR':{pose['rear_leg_right']:.0f},'FR':{pose['front_leg_right']:.0f}}}")

        time.sleep(max(0, DT - (time.time() - loop_start)))


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        fold_to_crouch_and_release("keyboard interrupt")
    except Exception as e:
        log(f"ERROR: {e}")
        fold_to_crouch_and_release(f"exception: {e}")
        raise
