import os
import sys
import time
os.system("cd /usr/bin && sudo rm python && sudo ln -s python3 python")
flag=0x00
for x in range(1,4):
    if os.system("sudo apt-get update") == 0:
        flag=flag | 0x01
        break
for x in range(1,4):
    if os.system("sudo pip3 install rpi_ws281x") == 0:
        flag=flag | 0x02
        break
for x in range(1,4):
    if os.system("sudo pip3 install mpu6050-raspberrypi") == 0:
        flag=flag | 0x04
        break
for x in range(1,4):
    if os.system("sudo apt-get install -y libqt5gui5 python3-dev python3-pyqt5 ") == 0:
        flag=flag | 0x08
        break
if flag==0x0F:
        print("\nNow the installation is successful.")
        print("\nPlease restart raspberry pi")
else:
    print ("\nSome libraries have not been installed yet. Please run 'sudo python setup.py' again")

