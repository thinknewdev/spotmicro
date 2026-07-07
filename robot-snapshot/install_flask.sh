#!/bin/bash

echo "🚀 Starting Flask Backend Installation on Raspberry Pi..."

# Update system packages
echo "🔄 Updating package list..."
sudo apt update -y && sudo apt upgrade -y

# Install Python3, pip, and virtual environment tools
echo "🐍 Installing Python3 and required packages..."
sudo apt install -y python3 python3-pip python3-venv git

# Create a new directory for the Flask backend
echo "📂 Setting up backend directory..."
mkdir -p ~/spotmicro_flask
cd ~/spotmicro_flask

# Set up a virtual environment
echo "🌐 Creating Python virtual environment..."
python3 -m venv venv
source venv/bin/activate

# Install required Python packages
echo "📦 Installing Python dependencies..."
pip install flask flask-socketio flask-cors adafruit-circuitpython-pca9685 mpu6050-raspberrypi

# Create the Flask backend script
echo "📝 Writing Flask server script..."
cat <<EOF > app.py
from flask import Flask, jsonify, request
from flask_socketio import SocketIO
from flask_cors import CORS
import time
import subprocess
from mpu6050 import mpu6050
import busio
from board import SCL, SDA
from adafruit_pca9685 import PCA9685
import threading

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

def send_sensor_data():
    while True:
        accel = mpu.get_accel_data()
        gyro = mpu.get_gyro_data()
        try:
            output = subprocess.check_output(["sudo", "python3", "adeept_HAT/RobotHAT/02_ultra.py"])
            distance = float(output.decode().strip())
        except:
            distance = None
        
        servos = {name: pwm.channels[channel].duty_cycle for name, channel in servo_channels.items()}
        
        socketio.emit("sensor_update", {
            "mpu6050": {"acceleration": accel, "gyroscope": gyro},
            "ultrasonic": {"distance_cm": distance},
            "servos": servos
        })
        
        time.sleep(0.5)

@socketio.on("connect")
def handle_connect():
    print("Client connected")

@socketio.on("disconnect")
def handle_disconnect():
    print("Client disconnected")

@socketio.on("set_servo")
def set_servo(data):
    servo_name = data.get("servo")
    angle = data.get("angle")
    if servo_name in servo_channels:
        channel = servo_channels[servo_name]
        duty_cycle = int((angle / 180) * 65535)
        pwm.channels[channel].duty_cycle = duty_cycle
        socketio.emit("servo_update", {servo_name: angle})
    else:
        socketio.emit("error", {"message": "Invalid servo name"})

sensor_thread = threading.Thread(target=send_sensor_data, daemon=True)
sensor_thread.start()

if __name__ == "__main__":
    socketio.run(app, host="0.0.0.0", port=5000, debug=True)
EOF

# Create a systemd service to run Flask on startup
echo "🛠 Creating systemd service..."
sudo bash -c 'cat <<EOF > /etc/systemd/system/spotmicro_flask.service
[Unit]
Description=SpotMicro Flask Backend
After=network.target

[Service]
User=pi
WorkingDirectory=/home/pi/spotmicro_flask
ExecStart=/home/pi/spotmicro_flask/venv/bin/python /home/pi/spotmicro_flask/app.py
Restart=always

[Install]
WantedBy=multi-user.target
EOF'

# Enable and start the service
echo "🚀 Enabling and starting Flask service..."
sudo systemctl daemon-reload
sudo systemctl enable spotmicro_flask
sudo systemctl start spotmicro_flask

echo "✅ Flask backend installed and running!"
echo "🌐 You can now access it at: http://<your-raspberry-pi-ip>:5000"
