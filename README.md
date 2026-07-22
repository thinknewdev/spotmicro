# SpotMicro

A working quadruped robot dog built on the [SpotMicroAI](https://spotmicroai.readthedocs.io/en/latest/)
community frame, running on a Raspberry Pi. Twelve servos, an IMU, an ultrasonic
sensor, two camera feeds, and a motion daemon that can stand, walk, turn, sit,
wave, dance, hop, and limp on three legs when a servo gives out.

Everything here is the real deployed system: the calibration numbers are measured
off the physical robot, and the gait parameters are the ones that were tuned until
the robot actually walked.

## What it does

| Command   | Behavior |
|-----------|----------|
| `stand`   | Rise from rest into a balanced stance and hold it with IMU roll correction |
| `walk`    | Diagonal-pair trot for N cycles, with steering and forward/reverse |
| `left` / `right` | Walk shorthands with a preset turn bias |
| `crawl`   | IK-driven creep gait (one foot at a time, three-point support) |
| `limp`    | Injured-leg gait: rear-right held rigid as a peg, weight biased off it, the three good legs walk one at a time |
| `hop`     | Four-leg launch with resonant preload, rear-first push, soft-crouch landing |
| `sit`     | Rear legs tucked, front legs bracing the raised chest |
| `wave`    | Sit, then raise the front-right paw and wag it |
| `dance`   | Bounce / sway / rock / groove moves, or the full routine |
| `stop`    | Abort whatever is running and ease back to a stable stand |
| `rest`    | Fold to the ground and de-energize the servos |
| `calpose` | Hold a single joint at a known angle for calibration |
| `pawtest` | Sweep one leg through its IK envelope |

## Hardware

- Raspberry Pi (64-bit Raspberry Pi OS), user `spotmicro`
- PCA9685 16-channel PWM driver at I2C `0x40`, 50 Hz, 25 MHz reference clock
- 12x servos: shoulder / thigh / knee per leg
- MPU6050 accelerometer + gyro at I2C `0x68`
- HC-SR04 ultrasonic rangefinder (BCM 11 trigger, BCM 8 echo)
- Pi camera plus an external USB webcam, both published over WebRTC
- 5V UBEC for servo power. This is the jump ceiling: the hop height is
  limited by current delivery, not by the gait.

Measured leg geometry: thigh (hip to knee) 11.0 cm, shank (knee to ankle) 13.5 cm,
foot pad contact at 14.5 cm.

## Architecture

Three processes, split so that nothing but the motion daemon ever touches I2C
during motion.

```
  browser (phone / laptop)
        |
        |  HTTP :5000, WebRTC :8889
        v
  +-------------------+        /tmp/spot_cmd (JSON)        +------------------+
  |  Flask API        |  --------------------------------> |  spot_motiond    |
  |  spotmicro_flask  |  <-------------------------------- |  (systemd)       |
  |  :5000, WS :5001  |   /tmp/spot_status.json            |                  |
  +-------------------+   /tmp/spot_servos.json            +--------+---------+
        |                                                           |
        | GPIO (ultrasonic)                                         | I2C
        v                                                           v
   HC-SR04                                          PCA9685 servos + MPU6050
```

**`spot_motiond.py`** is the motion daemon and the heart of the project. It owns
every servo and IMU read, and runs as a state machine (`REST` <-> `STAND` <->
`WALK` / `SIT` / `DANCE` / ...) at a fixed 20 Hz. Commands arrive as JSON written
atomically to `/tmp/spot_cmd`; status and live commanded angles are published back
to `/tmp/spot_status.json` and `/tmp/spot_servos.json`. Touching `/tmp/spot_stop`
folds the robot and exits.

**`spotmicro_flask/app.py`** is a thin control surface. It forwards motion commands
to the daemon through the command file, serves the web panel, reads the ultrasonic
sensor over pure GPIO (safe to poll during motion), and streams IMU data over
WebSocket. It never drives a servo while the daemon is running.

**`control-panel/`** is the web UI. `build_panel.py` holds a single HTML template
and emits two builds from it: `control-panel/index.html` for a laptop on the LAN
(hardcoded robot IP) and `robot-snapshot/spotmicro_flask/static/control.html`
served by the Pi itself (uses `location.hostname`). Edit the template, rerun the
script, never edit the generated files.

## Repository layout

```
control-panel/            Web control panel served from a laptop
  build_panel.py            Template + builder for both panel variants
  index.html                Generated. Do not edit directly.
  start.sh                  Serves the panel on :8080

robot-snapshot/           Snapshot of /home/spotmicro on the Pi
  spot_motiond.py           Motion daemon (state machine, gaits, IK, guards)
  spot_motiond.GOLDEN-approved-trot.py
                            Locked-in known-good walk. Restore point.
  stand_hold.py             Standalone stand-and-balance script
  robot_calibration.json    Measured pose, gait, and controller constants
  standing_reference.json   Reference standing angles
  spotmicroai.json          Servo channel map and pulse ranges
  spotmicro_flask/          Flask API + WebSocket + static panel
  spot_micro_kinematics/    Leg IK / FK library
  calibration/              Interactive per-servo calibration tools
  integration_tests/        Motion, abort, remote, and LCD checks
  adeept_HAT/               Vendor HAT sample code
  _system/                  systemd units and runtime notes
  install_flask.sh          Flask backend bootstrap
  install_mediamtx.sh       WebRTC/RTSP media server bootstrap
```

`spotmicroai/` is the upstream community runtime, kept locally but not tracked
here. It has its own history at
[gitlab.com/custom_robots/spotmicroai](https://gitlab.com/custom_robots/spotmicroai).

## Motion API

All motion endpoints accept `GET` or `POST` on port 5000.

```
POST /motion/stand
POST /motion/walk?cycles=8&turn=0.0&dir=1
POST /motion/left            # walk with turn=-0.8
POST /motion/right           # walk with turn=+0.8
POST /motion/crawl?cycles=4
POST /motion/limp?cycles=6
POST /motion/hop
POST /motion/sit
POST /motion/wave
POST /motion/dance?move=bounce|sway|rock|groove|full
POST /motion/stop
POST /motion/rest
POST /motion/calpose?n=0
GET  /motion/status          # daemon state, tilt, throttle flags
```

Parameters: `cycles` (default 8), `turn` in `[-1, 1]`, `dir` 1 forward or -1
reverse, `move` for dance variants, `n` to select a joint for `calpose`.

Sensor and telemetry endpoints:

```
GET  /                       # web control panel
GET  /servos                 # live commanded angles + state (file read, no I2C)
GET  /proximity              # ultrasonic distance in cm
GET  /sensor-data            # IMU + ultrasonic + servo angles
POST /set-servo              # {"servo": "front_leg_left", "angle": 90}
ws://<robot>:5001            # IMU/ultrasonic/servo stream at 2 Hz
```

Camera feeds are served by mediamtx on port 8889: `/cam/` for the onboard camera
and `/webcam/` for the external one.

## Setup

On the Pi:

```bash
# Flask backend and dependencies
./install_flask.sh

# WebRTC / RTSP media server for the camera feeds
./install_mediamtx.sh

# Install the services
sudo cp _system/spot_motiond.service /etc/systemd/system/
sudo cp _system/spotcam-push.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now spot_motiond spotmicro_flask
```

The motion daemon expects `~/spotmicroai.json` (channel map, pulse ranges) and
`~/spotmicroai/robot_calibration.json` (pose and gait constants). Both are in
`robot-snapshot/`.

From a laptop on the same network:

```bash
cd control-panel
./start.sh          # http://localhost:8080
```

Set the robot's IP at the bottom of `build_panel.py` and rerun it to regenerate
both panels.

Logs:

```bash
tail -f /var/log/spot_motiond.log
tail -f /var/log/spotmicro.log
```

## Calibration notes

These were expensive to learn. They are worth reading before changing anything.

**Channel map.** `front_shoulder_left` and `front_leg_left` were crossed in the
stock `spotmicroai.json`. They are channels 7 and 6 respectively. A crossed
channel makes the robot appear to have a wildly miscalibrated joint when the
angles are actually fine.

**Mirrored conventions.** Left and right legs run opposite sign conventions
(`THIGHS` and `KNEES` in the daemon carry the per-joint sign). Any correction
applied symmetrically to a left/right pair twists the body instead of leveling
it. This is why balance correction is roll-only: pitch correction visibly twists
the frame.

**Shoulders stay pinned.** Only thighs and knees move through stand, fold, and
gait. Shoulders hold their calibrated angle in every phase.

**Per-joint trim.** A replaced servo often sits a few horn splines off. Rather
than patching every pose, `TRIM` in `spot_motiond.py` applies an offset at the
single write point, so rest, sit, stand, and gait are all corrected at once.

**Tilt lies during motion.** Accelerometer tilt is meaningless mid-stride. The
tip guard uses a 3-second rolling mean above 15 degrees, not an instantaneous
reading.

**The gait stutter was `vcgencmd`.** Polling throttle state inside the motion
loop cost 20 to 50 ms per call. It is now cached on a background thread at 1 Hz.
Nothing that can block belongs in the loop.

**Balance controller:** 20 Hz loop, kp 0.15, 4 degree deadband, 0.9 degrees per
cycle slew limit, 8 degree offset clamp.

**Approved trot:** swing forward 13.0, swing back 17.0, lift 26.0, push extension
11.0, cycle time 2.0 s, walk sway 3.5, arm bob 4.0, symmetric sine lift curve.
Turning scales the inner side by `1 - 2*|turn|` with a floor of -0.5, so a full
turn drives the inner strides backward into a pivot.

**Known hardware fault.** The rear-right corner is chronically weak and its foot
scuffs on the swing. Software remedies are exhausted (it already gets a 10 degree
lift bias). The `limp` gait exists because of it: pegged rear-right, weight
shifted onto the left, and the robot still moves.

## Safety

The robot can fall. It is heavy enough and rigid enough to damage itself and
whatever it lands on.

- Keep a clear fall radius and watch it while it moves. Do not run gaits
  unattended.
- `hop` and fast turns are the highest-risk moves. Have `stop` ready.
- `touch /tmp/spot_stop` folds it and kills the daemon from any shell.
- Undervoltage aborts are built in: the daemon reads the throttle flags and folds
  to rest rather than browning out mid-stride.

## Credit

Built on the SpotMicroAI community project. Frame, kinematics library, and base
runtime come from [gitlab.com/custom_robots/spotmicroai](https://gitlab.com/custom_robots/spotmicroai).
The motion daemon, gaits, calibration, control panels, and camera pipeline in
this repository are original work.

MIT licensed, following upstream.
