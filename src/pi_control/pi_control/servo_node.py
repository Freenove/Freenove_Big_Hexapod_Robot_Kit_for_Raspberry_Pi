import rclpy
from rclpy.node import Node
import time
import sys
import math
from pathlib import Path

from .pca9685 import PCA9685
from .adc import ADC
from .imu import IMU
from robot_interfaces.srv import SetServo
from sensor_msgs.msg import JointState, BatteryState, Imu

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
            self.adc = ADC()
            self.imu = IMU()
            self.logger.info("Successfully initialized ADC monitor 0x84.")
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
        self.battery1_publisher = self.create_publisher(BatteryState, 'battery1_state', 10)
        self.battery2_publisher = self.create_publisher(BatteryState, 'battery2_state', 10)
        self.imu_publisher = self.create_publisher(Imu, 'imu/data_raw', 10)

        self.publish_timer = self.create_timer(0.05, self.publish_joint_states)
        self.publish_timer = self.create_timer(1.0, self.publish_battery_states)
        self.publish_timer = self.create_timer(0.1, self.publish_imu_data)

        self.set_angle_service = self.create_service(
            SetServo,
            'set_servo_angle',
            self.set_servo_angle_callback)

        self.initialize_servos()

    def publish_imu_data(self):
        """Read IMU data and publish Imu messages."""
        try:
            self.servo_controller.imu.update_imu_state()
            
            accel_data = self.servo_controller.imu.latest_linear_acceleration
            gyro_data = self.servo_controller.imu.latest_angular_velocity
            
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = 'imu_link'

            imu_msg.linear_acceleration.x = accel_data['x']
            imu_msg.linear_acceleration.y = accel_data['y']
            imu_msg.linear_acceleration.z = accel_data['z']

            imu_msg.angular_velocity.x = math.radians(gyro_data['x'])
            imu_msg.angular_velocity.y = math.radians(gyro_data['y'])
            imu_msg.angular_velocity.z = math.radians(gyro_data['z'])

            imu_msg.orientation.w = self.servo_controller.imu.quaternion_w
            imu_msg.orientation.x = self.servo_controller.imu.quaternion_x
            imu_msg.orientation.y = self.servo_controller.imu.quaternion_y
            imu_msg.orientation.z = self.servo_controller.imu.quaternion_z

            self.imu_publisher.publish(imu_msg)
        except Exception as e:
            self.get_logger().error(f"Failed to read or publish IMU data: {e}")
    
    def publish_battery_states(self):
        """Read battery voltages and publish BatteryState messages.
        Battery percentage is calculated based on voltage range 6.0V (empty) 
        to 8.4V (full) for a 2S Li-ion battery.
        """
        try:
            battery1_voltage, battery2_voltage = self.servo_controller.adc.read_battery_voltage()
            
            battery1_msg = BatteryState()
            battery1_msg.voltage = battery1_voltage
            battery1_msg.percentage = min(max((battery1_voltage - 6.0) / (8.4 - 6.0), 0.0), 1.0)
            battery1_msg.header.stamp = self.get_clock().now().to_msg()
            self.battery1_publisher.publish(battery1_msg)

            battery2_msg = BatteryState()
            battery2_msg.voltage = battery2_voltage
            battery2_msg.percentage = min(max((battery2_voltage - 6.0) / (8.4 - 6.0), 0.0), 1.0)
            battery2_msg.header.stamp = self.get_clock().now().to_msg()
            self.battery2_publisher.publish(battery2_msg)
            self.get_logger().info(f"Battery1 Voltage: {battery1_voltage}V, Battery2 Voltage: {battery2_voltage}V")
        except Exception as e:
            self.get_logger().error(f"Failed to read or publish battery states: {e}")

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