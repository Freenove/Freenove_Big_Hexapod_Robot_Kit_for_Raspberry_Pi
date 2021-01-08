#coding:utf-8
import Adafruit_PCA9685
import time 
import math
import smbus
class Servo:
    def __init__(self):
        self.pwm_40 = Adafruit_PCA9685.PCA9685(0x40)
        self.pwm_41 = Adafruit_PCA9685.PCA9685(0x41) 
        # Set the cycle frequency of PWM  
        self.pwm_40.set_pwm_freq(50) 
        time.sleep(0.01) 
        self.pwm_41.set_pwm_freq(50) 
        time.sleep(0.01)             

    #Convert the input angle to the value of pca9685
    def setServoAngle(self,channel, angle):  
        if channel < 16:
            date = 4096 * ((angle * 10) + 850) / 20000
            self.pwm_41.set_pwm(channel, 0, int(date))
        elif channel >= 16 and channel < 32:
            channel-=16
            date = 4096 * ((angle * 10) + 850) / 20000
            self.pwm_40.set_pwm(channel, 0, int(date))
        #time.sleep(0.0001)
    def relax(self):
        for i in range(8):
            self.pwm_41.set_pwm(i+8, 4096, 4096)
            self.pwm_40.set_pwm(i, 4096, 4096)
            self.pwm_40.set_pwm(i+8, 4096, 4096)

# Main program logic follows:
if __name__ == '__main__':
    print("Now servos will rotate to certain angles.") 
    print("Please keep the program running when installing the servos.")
    print("After that, you can press ctrl-C to end the program.")
    S=Servo()
    while True:
        try:
            for i in range(32):
                S.setServoAngle(i,90)
        except KeyboardInterrupt:
            print ("\nEnd of program")
        S.setServoAngle(13,0)
        S.setServoAngle(10,0)
        S.setServoAngle(31,0)
        S.setServoAngle(18,180)
        S.setServoAngle(21,180)
        S.setServoAngle(27,180)
        break


