# servo_controller/servo_controller/servo_node.py

import rclpy
from rclpy.node import Node
import time
import sys
from pathlib import Path

# Import the PCA9685 driver and the custom service types
from .pca9685 import PCA9685
from robot_interfaces.srv import SetServo, GetServo

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
        # The duty cycle for a standard servo is typically 500us (0 deg) to 2500us (180 deg)
        # The total PWM period at 50Hz is 20000us.
        pulse_us = map_value(angle, 0, 180, 500, 2500)
        
        # Convert pulse width in microseconds to a 12-bit duty cycle value (0-4095)
        # (pulse_us / 20000us) * 4096
        duty_cycle = int((pulse_us / 20000.0) * 4095.0)

        if channel < 16:
            if self.pwm_41: self.pwm_41.set_pwm(channel, 0, duty_cycle)
        elif 16 <= channel < 32:
            if self.pwm_40: self.pwm_40.set_pwm(channel - 16, 0, duty_cycle)

    def relax(self):
        """Relax all servos by turning off PWM."""
        if self.pwm_40 and self.pwm_41:
            for i in range(16):
                # Setting OFF register to 4096 fully turns off the channel
                self.pwm_40.set_pwm(i, 0, 4096)
                self.pwm_41.set_pwm(i, 0, 4096)
            self.logger.info("All servos relaxed.")

class ServoNode(Node):
    def __init__(self):
        super().__init__('servo_node')
        self.get_logger().info("Servo Control Node has started.")

        # Initialize the Servo controller class
        self.servo_controller = Servo(self.get_logger())

        # Store the last commanded angle for each servo (0-31)
        # Initialize all to 90 degrees as a safe neutral position
        self.servo_angles = [90] * 32

        # Create the services
        self.set_angle_service = self.create_service(
            SetServo,
            'set_servo_angle',
            self.set_servo_angle_callback)
            
        self.get_angle_service = self.create_service(
            GetServo,
            'get_servo_angle',
            self.get_servo_angle_callback)

        # Set all servos to their initial position
        self.initialize_servos()

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

    def get_servo_angle_callback(self, request, response):
        """Callback for the get_servo_angle service."""
        channel = request.channel
        
        if not (0 <= channel < 32):
            self.get_logger().error(f"Invalid channel requested: {channel}.")
            response.angle = -1 # Return -1 for invalid channel
            return response
            
        response.angle = self.servo_angles[channel]
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
        # Cleanly shut down the node and relax servos
        servo_node.on_shutdown()
        servo_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()