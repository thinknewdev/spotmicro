from flask import Flask, jsonify
from flask_socketio import SocketIO
from flask_cors import CORS
import time
import threading
import RPi.GPIO as GPIO
from mpu6050 import mpu6050
import busio
from board import SCL, SDA
from adafruit_pca9685 import PCA9685

# Initialize Flask app
app = Flask(__name__)
CORS(app)
socketio = SocketIO(app, cors_allowed_origins="*")

# Initialize MPU6050 Sensor
mpu = mpu6050(0x68)

# Initialize PCA9685 (Servo Driver)
i2c = busio.I2C(SCL, SDA)
pwm = PCA9685(i2c)
pwm.frequency = 50

# Servo Configuration
servo_channels = {
    "rear_shoulder_left": 0,
    "rear_leg_left": 1,
    "rear_feet_left": 2,
    "rear_shoulder_right": 3,
    "rear_leg_right": 4,
    "rear_feet_right": 5,
    "front_shoulder_left": 6,
    "front_leg_left": 7,
    "front_feet_left": 8,
    "front_shoulder_right": 9,
    "front_leg_right": 10,
    "front_feet_right": 11
}

# GPIO Setup for Ultrasonic Sensor
TRIGGER_PIN = 11
ECHO_PIN = 8

GPIO.setwarnings(False)
GPIO.setmode(GPIO.BCM)
GPIO.setup(TRIGGER_PIN, GPIO.OUT, initial=GPIO.LOW)
GPIO.setup(ECHO_PIN, GPIO.IN)

# Helper function to read distance from Ultrasonic Sensor
def check_distance():
    GPIO.output(TRIGGER_PIN, GPIO.HIGH)
    time.sleep(0.000015)
    GPIO.output(TRIGGER_PIN, GPIO.LOW)

    while not GPIO.input(ECHO_PIN):
        pass
    t1 = time.time()

    while GPIO.input(ECHO_PIN):
        pass
    t2 = time.time()

    return round((t2 - t1) * 340 / 2 * 100, 2)  # Convert to cm

# Function to fetch current sensor data
def get_sensor_data():
    accel = mpu.get_accel_data()
    gyro = mpu.get_gyro_data()
    distance = check_distance()
    servos = {name: pwm.channels[channel].duty_cycle for name, channel in servo_channels.items()}

    return {
        "mpu6050": {"acceleration": accel, "gyroscope": gyro},
        "ultrasonic": {"distance_cm": distance},
        "servos": servos
    }

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

    if servo_name in servo_channels and 0 <= angle <= 180:
        channel = servo_channels[servo_name]
        duty_cycle = int((angle / 180) * 65535)  # Convert angle to PWM
        pwm.channels[channel].duty_cycle = duty_cycle
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

    if servo_name in servo_channels and 0 <= angle <= 180:
        channel = servo_channels[servo_name]
        duty_cycle = int((angle / 180) * 65535)
        pwm.channels[channel].duty_cycle = duty_cycle
        socketio.emit("servo_update", {servo_name: angle})  # Notify all clients
    else:
        socketio.emit("error", {"message": "Invalid servo name or angle"})

# Start WebSocket data streaming in a background thread
sensor_thread = threading.Thread(target=send_sensor_data, daemon=True)
sensor_thread.start()

# Run Flask with WebSocket support
if __name__ == "__main__":
    socketio.run(app, host="0.0.0.0", port=5000, debug=False, allow_unsafe_werkzeug=True)
