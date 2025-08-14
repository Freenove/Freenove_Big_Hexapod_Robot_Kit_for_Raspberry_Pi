# coding:utf-8
from pca9685 import PCA9685
import time

def map_value(value, from_low, from_high, to_low, to_high):
    """Map a value from one range to another."""
    return (to_high - to_low) * (value - from_low) / (from_high - from_low) + to_low

class Servo:
    def __init__(self):
        self.pwm_40 = PCA9685(0x40, debug=True)
        self.pwm_41 = PCA9685(0x41, debug=True)
        # Set the cycle frequency of PWM to 50 Hz
        self.pwm_40.set_pwm_freq(50)
        time.sleep(0.01)
        self.pwm_41.set_pwm_freq(50)
        time.sleep(0.01)

    def set_servo_angle(self, channel, angle):
        """
        Convert the input angle to the value of PCA9685 and set the servo angle.
        
        :param channel: Servo channel (0-31)
        :param angle: Angle in degrees (0-180)
        """
        if channel < 16:
            duty_cycle = map_value(angle, 0, 180, 500, 2500)
            duty_cycle = map_value(duty_cycle, 0, 20000, 0, 4095)
            self.pwm_41.set_pwm(channel, 0, int(duty_cycle))
        elif channel >= 16 and channel < 32:
            channel -= 16
            duty_cycle = map_value(angle, 0, 180, 500, 2500)
            duty_cycle = map_value(duty_cycle, 0, 20000, 0, 4095)
            self.pwm_40.set_pwm(channel, 0, int(duty_cycle))

    def relax(self):
        """Relax all servos by setting their PWM values to 4096."""
        for i in range(8):
            self.pwm_41.set_pwm(i + 8, 4096, 4096)
            self.pwm_40.set_pwm(i, 4096, 4096)
            self.pwm_40.set_pwm(i + 8, 4096, 4096)


# Main program logic follows:
if __name__ == '__main__':
    print("Now servos will rotate to certain angles.")
    print("Please keep the program running when installing the servos.")
    print("After that, you can press ctrl-C to end the program.")
    servo = Servo()
    while True:
        try:
            for i in range(32):
                if i in [10, 13, 31]:
                    servo.set_servo_angle(i, 10)
                elif i in [18, 21, 27]:
                    servo.set_servo_angle(i, 170)
                else:
                    servo.set_servo_angle(i, 90)
            time.sleep(3)
        except KeyboardInterrupt:
            print("\nEnd of program")
            servo.relax()
            break