from flask import Flask, jsonify, request
from flask_socketio import SocketIO
from flask_cors import CORS
import asyncio
import websockets
import json
import time
import threading
import RPi.GPIO as GPIO
from mpu6050 import mpu6050
import busio
from board import SCL, SDA
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
import sys
sys.path.insert(0, '/home/spotmicro/spotmicroai')
from spotmicroai.utilities.config import Config

# Initialize Flask app
app = Flask(__name__)
CORS(app)
socketio = SocketIO(app, cors_allowed_origins="*")

# Initialize MPU6050 Sensor
mpu = mpu6050(0x68)

# Initialize I2C and Config
i2c = busio.I2C(SCL, SDA)
config = Config()

# Servo names
servo_names = [
    "rear_shoulder_left", "rear_leg_left", "rear_feet_left",
    "rear_shoulder_right", "rear_leg_right", "rear_feet_right",
    "front_shoulder_left", "front_leg_left", "front_feet_left",
    "front_shoulder_right", "front_leg_right", "front_feet_right"
]

# Initialize PCA9685 controllers and servos with proper calibration
pca_controllers = {}
servos = {}

for servo_name in servo_names:
    try:
        PCA9685_ADDRESS, PCA9685_REFERENCE_CLOCK_SPEED, PCA9685_FREQUENCY, CHANNEL, MIN_PULSE, MAX_PULSE, REST_ANGLE = \
            config.get_by_section_name(servo_name)

        if PCA9685_ADDRESS not in pca_controllers:
            pca = PCA9685(i2c, address=int(PCA9685_ADDRESS, 0), reference_clock_speed=PCA9685_REFERENCE_CLOCK_SPEED)
            pca.frequency = PCA9685_FREQUENCY
            pca_controllers[PCA9685_ADDRESS] = pca
        else:
            pca = pca_controllers[PCA9685_ADDRESS]

        active_servo = servo.Servo(pca.channels[CHANNEL])
        active_servo.set_pulse_width_range(min_pulse=MIN_PULSE, max_pulse=MAX_PULSE)
        servos[servo_name] = active_servo
    except Exception as e:
        print(f"Error initializing {servo_name}: {e}")

# GPIO Setup for Ultrasonic Sensor
TRIGGER_PIN = 11
ECHO_PIN = 8

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIGGER_PIN, GPIO.OUT, initial=GPIO.LOW)
# Pull-down so a disconnected/unpowered sensor reads LOW and times out fast
GPIO.setup(ECHO_PIN, GPIO.IN, pull_up_down=GPIO.PUD_DOWN)

# Helper function to read distance from Ultrasonic Sensor
def check_distance():
    """Read HC-SR04 distance in cm. Returns None on timeout (sensor absent/miswired)."""
    GPIO.output(TRIGGER_PIN, GPIO.HIGH)
    time.sleep(0.000015)
    GPIO.output(TRIGGER_PIN, GPIO.LOW)

    t0 = time.time()
    while not GPIO.input(ECHO_PIN):
        if time.time() - t0 > 0.05:  # no echo rise within 50ms -> give up
            return None
    t1 = time.time()

    while GPIO.input(ECHO_PIN):
        if time.time() - t1 > 0.05:  # echo stuck high -> give up
            return None
    t2 = time.time()

    return round((t2 - t1) * 340 / 2 * 100, 2)  # Convert to cm

# Function to fetch current sensor data
def get_sensor_data():
    accel = mpu.get_accel_data()
    gyro = mpu.get_gyro_data()
    distance = check_distance()

    # Get actual servo angles using calibrated servo objects
    servo_angles = {}
    for name, servo_obj in servos.items():
        try:
            angle = servo_obj.angle
            # angle is None when the channel is de-energized (rest mode)
            servo_angles[name] = round(angle, 2) if angle is not None else None
        except Exception as e:
            print(f"Error reading {name}: {e}")
            servo_angles[name] = None

    return {
        "mpu6050": {"acceleration": accel, "gyroscope": gyro},
        "ultrasonic": {"distance_cm": distance},
        "servos": servo_angles
    }

# 🐕 Motion control (spot_motiond daemon owns all motion; we just send commands)
import os as _os
from flask import send_from_directory
MOTION_CMD_FILE = "/tmp/spot_cmd"
MOTION_STATUS_FILE = "/tmp/spot_status.json"

@app.route("/")
def control_ui():
    """Web control panel (static/control.html)"""
    return send_from_directory(_os.path.join(_os.path.dirname(_os.path.abspath(__file__)), "static"),
                               "control.html")

@app.route("/servos", methods=["GET"])
def servo_positions():
    """Live commanded servo angles from spot_motiond (no I2C — file read)"""
    try:
        with open("/tmp/spot_servos.json") as f:
            return jsonify(json.load(f))
    except Exception:
        return jsonify({"servos": {}, "state": "?"}), 200

@app.route("/proximity", methods=["GET"])
def proximity():
    """Ultrasonic distance only — pure GPIO, no I2C, safe to poll during motion"""
    return jsonify({"distance_cm": check_distance(), "ts": time.time()})

@app.route("/motion/<cmd>", methods=["GET", "POST"])
def motion_cmd(cmd):
    """stand | walk[?cycles=N] | rest | status — forwarded to spot_motiond"""
    if cmd == "status":
        try:
            with open(MOTION_STATUS_FILE) as f:
                return jsonify(json.load(f))
        except Exception:
            return jsonify({"error": "no status (spot_motiond not running?)"}), 503
    # left/right are walk shorthands with a preset turn value
    turn = request.args.get("turn", default=0.0, type=float)
    if cmd == "left":
        cmd, turn = "walk", -0.8
    elif cmd == "right":
        cmd, turn = "walk", 0.8
    if cmd not in ("stand", "walk", "rest", "sit", "wave", "dance", "crawl", "pawtest", "calpose", "stop", "hop", "limp"):
        return jsonify({"error": "unknown motion command"}), 400
    cycles = request.args.get("cycles", default=8, type=int)
    direction = -1 if request.args.get("dir", default=1, type=int) < 0 else 1
    turn = max(-1.0, min(1.0, turn))
    tmp = MOTION_CMD_FILE + ".tmp"
    with open(tmp, "w") as f:
        json.dump({"cmd": cmd, "cycles": cycles, "turn": turn, "dir": direction,
                   "n": request.args.get("n", default=0, type=int),
                   "move": request.args.get("move", default="full"), "ts": time.time()}, f)
    _os.replace(tmp, MOTION_CMD_FILE)
    return jsonify({"status": "sent", "cmd": cmd, "cycles": cycles, "turn": turn, "dir": direction})

# 🌍 REST API: Fetch Sensor Data
@app.route("/sensor-data", methods=["GET"])
def get_sensor_data_rest():
    """ REST API to get sensor readings """
    return jsonify(get_sensor_data())

# 🌍 REST API: Control Servo
@app.route("/set-servo", methods=["POST"])
def set_servo_rest():
    """ Set a servo angle via REST API """
    data = request.json
    servo_name = data.get("servo")
    angle = data.get("angle")

    if servo_name in servos and angle is not None and 0 <= angle <= 180:
        servos[servo_name].angle = angle  # uses calibrated pulse range
        return jsonify({"status": "success", "servo": servo_name, "angle": angle})

    return jsonify({"status": "error", "message": "Invalid servo name or angle"}), 400

# ⚡ WebSocket: Broadcast Sensor Data in Real-Time
def send_sensor_data():
    while True:
        socketio.emit("sensor_update", get_sensor_data())
        time.sleep(0.5)

@socketio.on("connect")
def handle_connect():
    print("Client connected")
    socketio.emit("message", {"status": "Connected to WebSocket"})

@socketio.on("disconnect")
def handle_disconnect():
    print("Client disconnected")

@socketio.on("set_servo")
def set_servo_ws(data):
    """ Set a servo angle via WebSocket """
    servo_name = data.get("servo")
    angle = data.get("angle")

    if servo_name in servos and angle is not None and 0 <= angle <= 180:
        servos[servo_name].angle = angle  # uses calibrated pulse range
        socketio.emit("servo_update", {servo_name: angle})  # Notify all clients
    else:
        socketio.emit("error", {"message": "Invalid servo name or angle"})

# Start WebSocket data streaming in a background thread
sensor_thread = threading.Thread(target=send_sensor_data, daemon=True)
sensor_thread.start()

# Plain WebSocket Handler for Dashboard (Port 5001)
async def websocket_handler(websocket):
    try:
        while True:
            sensor_data = get_sensor_data()
            await websocket.send(json.dumps(sensor_data))
            await asyncio.sleep(0.5)
    except websockets.exceptions.ConnectionClosedError as e:
        print(f"WebSocket connection closed: {e}")
    except Exception as e:
        print(f"WebSocket error: {e}")

async def start_websocket_server():
    async with websockets.serve(websocket_handler, "0.0.0.0", 5001):
        print("WebSocket server started on ws://0.0.0.0:5001")
        await asyncio.Future()

def run_websocket():
    loop = asyncio.new_event_loop()
    asyncio.set_event_loop(loop)
    loop.run_until_complete(start_websocket_server())

# Start plain WebSocket server in background thread
websocket_thread = threading.Thread(target=run_websocket, daemon=True)
websocket_thread.start()

# Run Flask with SocketIO support
if __name__ == "__main__":
    print("Starting Flask API on http://0.0.0.0:5000")
    print("Starting WebSocket on ws://0.0.0.0:5001")
    socketio.run(app, host="0.0.0.0", port=5000, debug=False, allow_unsafe_werkzeug=True)
