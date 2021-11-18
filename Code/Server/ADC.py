import smbus
import time
from ADCDevice import *

class ADC:
    def __init__(self):
        self.adcFlag = None
        self.adc = ADCDevice()
        if(self.adc.detectI2C(0x4f)): # Detect the pcf8591.
            self.adcFlag = False
            self.adc = PCF8591()
        elif(self.adc.detectI2C(0x48)): # Detect the ads7830
            self.adcFlag = True
            self.adc = ADS7830()
        else:
            print("No correct I2C address found, \n"
            "Please use command 'i2cdetect -y 1' to check the I2C address! \n"
            "Program Exit. \n");
            exit(-1)
    def batteryValue(self,chn):
        return self.adc.analogRead(chn)
        
    def batteryPower(self): 
        if self.adcFlag == True:
            val0 = self.batteryValue(0)
            val1 = self.batteryValue(4)
        else:
            val0 = self.batteryValue(0)
            val1 = self.batteryValue(1)
            
        battery1=round(val0/255*5*3,2)
        battery2=round(val1/255*5*3,2)
        #print(str(self.adc.address)+" "+str(val0)+" "+str(val1))
        return battery1,battery2


if __name__ == '__main__':
    print("start .. \n")
    adc=ADC()
    try:
        while True:
            Power=adc.batteryPower()
            print ("The battery voltage is "+str(Power)+'\n')
            time.sleep(1)
    except KeyboardInterrupt:
        adc.adc.close()
        print ("\nEnd of program")
    pass

