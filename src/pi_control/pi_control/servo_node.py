import rclpy
from rclpy.node import Node
import time
import sys
import math
from pathlib import Path

from .pca9685 import PCA9685
from robot_interfaces.srv import SetServo
from sensor_msgs.msg import JointState

JOINT_NAME_MAP = {
    0: 'camera_pan_joint',
    1: 'camera_tilt_joint',
    9: 'leg3_coxa_joint',
    8: 'leg3_femur_joint',
    31: 'leg3_tibia_joint',
    12: 'leg2_coxa_joint',
    11: 'leg2_femur_joint',
    10: 'leg2_tibia_joint',
    15: 'leg1_coxa_joint',
    14: 'leg1_femur_joint',
    13: 'leg1_tibia_joint',
    16: 'leg6_coxa_joint',
    17: 'leg6_femur_joint',
    18: 'leg6_tibia_joint',
    19: 'leg5_coxa_joint',
    20: 'leg5_femur_joint',
    21: 'leg5_tibia_joint',
    22: 'leg4_coxa_joint',
    23: 'leg4_femur_joint',
    27: 'leg4_tibia_joint',
}


def map_value(value, from_low, from_high, to_low, to_high):
    """Map a value from one range to another."""
    return (to_high - to_low) * (value - from_low) / (from_high - from_low) + to_low

class Servo:
    """A class to manage the two PCA9685 servo drivers."""
    def __init__(self, logger):
        self.logger = logger
        try:
            self.pwm_40 = PCA9685(0x40, debug=False)
            self.pwm_41 = PCA9685(0x41, debug=False)
            # Set the cycle frequency of PWM to 50 Hz
            self.pwm_40.set_pwm_freq(50)
            time.sleep(0.01)
            self.pwm_41.set_pwm_freq(50)
            time.sleep(0.01)
            self.logger.info("Successfully initialized PCA9685 drivers at 0x40 and 0x41.")
        except Exception as e:
            self.logger.error(f"Failed to initialize PCA9685 drivers: {e}")
            self.pwm_40 = None
            self.pwm_41 = None

    def set_servo_angle(self, channel, angle):
        """
        Convert the input angle to the value of PCA9685 and set the servo angle.
        
        :param channel: Servo channel (0-31)
        :param angle: Angle in degrees (0-180)
        """
        pulse_us = map_value(angle, 0, 180, 500, 2500)
        duty_cycle = int((pulse_us / 20000.0) * 4095.0)

        if channel < 16:
            if self.pwm_41: self.pwm_41.set_pwm(channel, 0, duty_cycle)
        elif 16 <= channel < 32:
            if self.pwm_40: self.pwm_40.set_pwm(channel - 16, 0, duty_cycle)

    def relax(self):
        """Relax all servos by turning off PWM."""
        if self.pwm_40 and self.pwm_41:
            for i in range(16):
                self.pwm_40.set_pwm(i, 0, 4096)
                self.pwm_41.set_pwm(i, 0, 4096)
            self.logger.info("All servos relaxed.")

class ServoNode(Node):
    def __init__(self):
        super().__init__('servo_node')
        self.get_logger().info("Servo Control Node has started.")

        self.servo_controller = Servo(self.get_logger())
        self.servo_angles = [90] * 32

        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)

        self.publish_timer = self.create_timer(0.05, self.publish_joint_states)

        self.set_angle_service = self.create_service(
            SetServo,
            'set_servo_angle',
            self.set_servo_angle_callback)

        self.initialize_servos()

    def publish_joint_states(self):
        """Create and publish a JointState message with all current servo angles."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = []
        msg.position = []

        for channel, name in JOINT_NAME_MAP.items():
            msg.name.append(name)
            
            angle_deg = self.servo_angles[channel]
            angle_rad = math.radians(angle_deg - 90.0)
            msg.position.append(angle_rad)

        self.joint_state_publisher.publish(msg)

    def initialize_servos(self):
        """Sets all servos to their initial default angle."""
        for i in range(32):
            self.servo_controller.set_servo_angle(i, self.servo_angles[i])
        self.get_logger().info("All servos set to initial position (90 degrees).")

    def set_servo_angle_callback(self, request, response):
        """Callback for the set_servo_angle service."""
        channel = request.channel
        angle = request.angle

        if not (0 <= channel < 32):
            response.success = False
            response.message = f"Invalid channel: {channel}. Must be 0-31."
            self.get_logger().error(response.message)
            return response

        if not (0 <= angle <= 180):
            response.success = False
            response.message = f"Invalid angle: {angle}. Must be 0-180."
            self.get_logger().error(response.message)
            return response

        self.get_logger().info(f"Setting servo {channel} to {angle} degrees.")
        self.servo_controller.set_servo_angle(channel, angle)
        self.servo_angles[channel] = angle # Store the new angle

        response.success = True
        response.message = f"Successfully set servo {channel} to {angle} degrees."
        return response


    def on_shutdown(self):
        """Called upon node shutdown."""
        self.get_logger().info("Shutting down, relaxing all servos...")
        self.servo_controller.relax()

def main(args=None):
    rclpy.init(args=args)
    servo_node = ServoNode()
    try:
        rclpy.spin(servo_node)
    except KeyboardInterrupt:
        pass
    finally:
        servo_node.on_shutdown()
        servo_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()