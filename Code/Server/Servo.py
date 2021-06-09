#coding:utf-8
import Adafruit_PCA9685
import time 
import math
import smbus
def mapNum(value,fromLow,fromHigh,toLow,toHigh):
    return (toHigh-toLow)*(value-fromLow) / (fromHigh-fromLow) + toLow
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
            date = mapNum(mapNum(angle,0,180,500,2500),0,20000,0,4095) # 0-180 map to 500-2500us ,then map to duty 0-4095
            self.pwm_41.set_pwm(channel, 0, int(date))
        elif channel >= 16 and channel < 32:
            channel-=16
            date = mapNum(mapNum(angle,0,180,500,2500),0,20000,0,4095) # 
            self.pwm_40.set_pwm(channel, 0, int(date))
        #time.sleep(0.0001)
    def relax(self):
        for i in range(8):
            self.pwm_41.set_pwm(i+8, 4096, 4096)
            self.pwm_40.set_pwm(i, 4096, 4096)
            self.pwm_40.set_pwm(i+8, 4096, 4096)
            
def servo_installation_position():
    S=Servo()     
    for i in range(32):
        if (i == 10 or i == 13 or i == 31):
            S.setServoAngle(i,0)
        elif (i == 18 or i == 21 or i == 27):
            S.setServoAngle(i,180)
        else:
            S.setServoAngle(i,90)
    time.sleep(3)
# Main program logic follows:
if __name__ == '__main__':
    print("Now servos will rotate to certain angles.") 
    print("Please keep the program running when installing the servos.")
    print("After that, you can press ctrl-C to end the program.")
    while True:
        try:        
            servo_installation_position()
        except KeyboardInterrupt:
            print ("\nEnd of program")
            break


