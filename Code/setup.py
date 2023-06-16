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
    if os.system("cd ./Libs/rpi-ws281x-python/library && sudo python3 setup.py install") == 0:
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
    if flag&0x01==0x00:
        print("\napt-get update failed.")
    if flag&0x02==0x00:
        print("\nrpi-ws281x-python install failed.")
    if flag&0x04==0x00:
        print("\nmpu6050-raspberrypi install failed.")
    if flag&0x08==0x00:
        print("\nlibqt5gui5 python3-dev python3-pyqt5 install failed.")
    print ("\nSome libraries have not been installed yet. Please run 'sudo python setup.py' again")

