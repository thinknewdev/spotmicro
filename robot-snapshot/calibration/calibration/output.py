#!/home/pi/spotmicroai/venv/bin/python3 -u

import busio
from board import SCL, SDA
from adafruit_pca9685 import PCA9685
from adafruit_motor import servo
from spotmicroai.utilities.config import Config
import time

# Initialize I2C bus
i2c = busio.I2C(SCL, SDA)

# Configuration object
config = Config()

# Servo options dictionary
servo_options = {
    0: ('rear_shoulder_left', Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_LEFT_PCA9685, Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_LEFT_CHANNEL),
    1: ('rear_leg_left', Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_LEFT_PCA9685, Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_LEFT_CHANNEL),
    2: ('rear_feet_left', Config.MOTION_CONTROLLER_SERVOS_REAR_FEET_LEFT_PCA9685, Config.MOTION_CONTROLLER_SERVOS_REAR_FEET_LEFT_CHANNEL),
    3: ('rear_shoulder_right', Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_RIGHT_PCA9685, Config.MOTION_CONTROLLER_SERVOS_REAR_SHOULDER_RIGHT_CHANNEL),
    4: ('rear_leg_right', Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_RIGHT_PCA9685, Config.MOTION_CONTROLLER_SERVOS_REAR_LEG_RIGHT_CHANNEL),
    5: ('rear_feet_right', Config.MOTION_CONTROLLER_SERVOS_REAR_FEET_RIGHT_PCA9685, Config.MOTION_CONTROLLER_SERVOS_REAR_FEET_RIGHT_CHANNEL),
    6: ('front_shoulder_left', Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_LEFT_PCA9685, Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_LEFT_CHANNEL),
    7: ('front_leg_left', Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_LEFT_PCA9685, Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_LEFT_CHANNEL),
    8: ('front_feet_left', Config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_LEFT_PCA9685, Config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_LEFT_CHANNEL),
    9: ('front_shoulder_right', Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_RIGHT_PCA9685, Config.MOTION_CONTROLLER_SERVOS_FRONT_SHOULDER_RIGHT_CHANNEL),
    10: ('front_leg_right', Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_RIGHT_PCA9685, Config.MOTION_CONTROLLER_SERVOS_FRONT_LEG_RIGHT_CHANNEL),
    11: ('front_feet_right', Config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_RIGHT_PCA9685, Config.MOTION_CONTROLLER_SERVOS_FRONT_FEET_RIGHT_CHANNEL),
    12: ('arm_rotation', Config.ARM_CONTROLLER_SERVOS_ARM_ROTATION_PCA9685, Config.ARM_CONTROLLER_SERVOS_ARM_ROTATION_CHANNEL),
    13: ('arm_lift', Config.ARM_CONTROLLER_SERVOS_ARM_LIFT_PCA9685, Config.ARM_CONTROLLER_SERVOS_ARM_LIFT_CHANNEL),
    14: ('arm_range', Config.ARM_CONTROLLER_SERVOS_ARM_RANGE_PCA9685, Config.ARM_CONTROLLER_SERVOS_ARM_RANGE_CHANNEL),
    15: ('arm_cam_tilt', Config.ARM_CONTROLLER_SERVOS_ARM_CAM_TILT_PCA9685, Config.ARM_CONTROLLER_SERVOS_ARM_CAM_TILT_CHANNEL),
}

def get_servo_config(servo_name):
    """Retrieve servo configuration details."""
    PCA9685_ADDRESS, PCA9685_REFERENCE_CLOCK_SPEED, PCA9685_FREQUENCY, CHANNEL, MIN_PULSE, MAX_PULSE, REST_ANGLE = \
        config.get_by_section_name(servo_name)
    return PCA9685_ADDRESS, PCA9685_REFERENCE_CLOCK_SPEED, PCA9685_FREQUENCY, CHANNEL, MIN_PULSE, MAX_PULSE, REST_ANGLE

def main():
    print("Servo Angle Monitoring Script")
    print("-----------------------------")

    # Dictionary to store initialized servos
    servos = {}

    # Initialize PCA9685 controllers
    pca_controllers = {}

    for index, (servo_name, pca_key, channel_key) in servo_options.items():
        try:
            # Get configuration for each servo
            PCA9685_ADDRESS, PCA9685_REFERENCE_CLOCK_SPEED, PCA9685_FREQUENCY, CHANNEL, MIN_PULSE, MAX_PULSE, REST_ANGLE = \
                get_servo_config(servo_name)

            # Check if PCA9685 controller already initialized
            if PCA9685_ADDRESS not in pca_controllers:
                pca = PCA9685(
                    i2c,
                    address=int(PCA9685_ADDRESS, 0),
                    reference_clock_speed=PCA9685_REFERENCE_CLOCK_SPEED
                )
                pca.frequency = PCA9685_FREQUENCY
                pca_controllers[PCA9685_ADDRESS] = pca
            else:
                pca = pca_controllers[PCA9685_ADDRESS]

            # Initialize servo
            active_servo = servo.Servo(pca.channels[CHANNEL])
            active_servo.set_pulse_width_range(min_pulse=MIN_PULSE, max_pulse=MAX_PULSE)
            servos[servo_name] = active_servo

        except Exception as e:
            print(f"Error initializing {servo_name}: {e}")

    try:
        while True:
            print("\nCurrent Servo Angles:")
            for servo_name, active_servo in servos.items():
                print(f"{servo_name}: {active_servo.angle}°")
            time.sleep(1)

    except KeyboardInterrupt:
        print("\nScript interrupted by user.")

    finally:
        for pca in pca_controllers.values():
            pca.deinit()
        print("PCA9685 controllers deinitialized.")

if __name__ == "__main__":
    main()