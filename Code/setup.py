import os
import sys
import time
os.system("cd /usr/bin && sudo rm python && sudo ln -s python3 python")
flag=0x00
for x in range(1,4):
	if os.system("sudo apt-get update") == 0:
		if os.system("sudo apt-get upgrade -y") == 0:
		    flag=flag | 0x01
		    break
for x in range(1,4):
	if os.system("sudo pip3 install adafruit-pca9685") == 0:
		flag=flag | 0x02
		break
for x in range(1,4):
	if os.system("sudo pip3 install mpu6050-raspberrypi") == 0:
		flag=flag | 0x04
		break
for x in range(1,4):
	if os.system("sudo apt-get install -y libqtgui4 python3-dev libqt4-test python3-pyqt5 ") == 0:
		flag=flag | 0x08
		break
for x in range(1,4):
	if os.system("sudo apt-get install -y libatlas-base-dev libjasper-dev") == 0:
		flag=flag | 0x10
		break
if flag==0x1F:
        print("\nNow the installation is successful.")
        print("\nPlease restart raspberry pi")
else:
	print ("\nSome libraries have not been installed yet. Please run 'sudo python setup.py' again")

