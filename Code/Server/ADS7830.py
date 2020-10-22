import smbus
import time
class ADS7830:
	def __init__(self):
		# Get I2C bus
		self.bus = smbus.SMBus(1)
		# I2C address of the device
		self.ADS7830_DEFAULT_ADDRESS			= 0x48
		# ADS7830 Command Set
		self.ADS7830_CMD				= 0x84 # Single-Ended Inputs
		self.battery1_flag = False
		self.battery2_flag = False
		self.battery1Voltage = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
		self.battery2Voltage = [0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
	def readAdc(self,channel):
		"""Select the Command data from the given provided value above"""
		COMMAND_SET = self.ADS7830_CMD | ((((channel<<2)|(channel>>1))&0x07)<<4)
		self.bus.write_byte(self.ADS7830_DEFAULT_ADDRESS, COMMAND_SET)
		data = self.bus.read_byte(self.ADS7830_DEFAULT_ADDRESS)
		return data
	def voltage(self,channel):
		if channel==0 or channel==4:
			data=[0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0]
			if self.battery1_flag == False or self.battery2_flag == False:
				for i in range(25):
					for j in range(25):
						data[j]=self.readAdc(channel)
					if channel==0:
						self.battery1Voltage.pop(0)
						self.battery1Voltage.append(max(data))
						battery_voltage=(sum(self.battery1Voltage)/len(self.battery1Voltage))/255.0*5.0
						self.battery1_flag = True
					else:
						self.battery2Voltage.pop(0)
						self.battery2Voltage.append(max(data))
						battery_voltage=(sum(self.battery2Voltage)/len(self.battery2Voltage))/255.0*5.0
						self.battery2_flag = True
			else:
				for j in range(25):
					data[j]=self.readAdc(channel)
				if channel==0:
					self.battery1Voltage.pop(0)
					self.battery1Voltage.append(max(data))
					battery_voltage=(sum(self.battery1Voltage)/len(self.battery1Voltage))/255.0*5.0
				else:
					self.battery2Voltage.pop(0)
					self.battery2Voltage.append(max(data))
					battery_voltage=(sum(self.battery2Voltage)/len(self.battery2Voltage))/255.0*5.0	
		else:
			data=[0,0,0,0,0,0,0,0,0]
			for i in range(9):
				data[i]=self.readAdc(channel)
			data.sort()
			battery_voltage=data[4]/255.0*5.0
		return battery_voltage
	def batteryPower(self):	
		battery1=round(self.voltage(0)*3,2)
		battery2=round(self.voltage(4)*3,2)
		return battery1,battery2
if __name__ == '__main__':
	pass

