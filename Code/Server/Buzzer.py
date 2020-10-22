import time
import RPi.GPIO as GPIO
class Buzzer:
    def __init__(self):
        GPIO.setwarnings(False)
        self.Buzzer_Pin = 17
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.Buzzer_Pin,GPIO.OUT)
    def run(self,command):
        if command!="0":
            GPIO.output(self.Buzzer_Pin,True)
        else:
            GPIO.output(self.Buzzer_Pin,False)
if __name__=='__main__':
    B=Buzzer()
    B.run('1')
    time.sleep(3)
    B.run('0')




