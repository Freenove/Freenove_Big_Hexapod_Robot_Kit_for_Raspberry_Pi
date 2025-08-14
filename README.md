# ROS2 implementation of Freenove Big Hexapod Robot kit for Raspberry Pi (FNK0052)

<img src='archive/Picture/icon.png' width='50%'/>

### Thank you to Freenova
Firstly, I'd like to thank the ppl at Freenova for being awesome and making applications like this available open source, no strings attached. This is the way. 🙏

### Starting Out
The best place to start is by following Freenova's kit instructions. This can be done by following the steps in this repos branch: <a href="[link](https://github.com/ogordillo/ROS2_Conversion_Freenove_Big_Hexapod_Robot_Raspberry_Pi/tree/original)">original</a> or by following the steps from Freenove project <a href="[link](https://github.com/Freenove/Freenove_Big_Hexapod_Robot_Kit_for_Raspberry_Pi)">Freenove_Big_Hexapod_Robot_Kit_for_Raspberry_Pi</a>

Get to the point where the robot is built and all the sensors are working (completing the Tutorial.pdf). Once you're there, you're ready for ROS! 🤖

## ROS2 Development

### Project Goals
This project aims to accomplish two core goals. 
- Recreate the Freenove provided functionality using ROS libraries and frameworks. 
- Have the Robot climb stairs. 

### Distributed Setup
With ROS at the core, this project will comprise of the following functionality
- windows_control: A pyqt5 application on a Windows PC (WSL2) to command the robot. 
- raspberry_pi: A Raspberry Pi 5 receives commands to actuate motors. 
- jetson_tx2: An nvidia Jetson TX2 streams depth data and runs AI models onboard 

This project uses CycloneDDS RMW (Middleware) for p2p discovery.

### Project Structure
- Project
  - windows_control
    - Dockerfile
    - src
  - raspberry_pi
    - Dockerfile
    - src
  - jetson_tx2
    - Dockerfile
    - src
    

### Version (Foxy)

I am personally working towards a ROS2 Foxy build because my Jetson TX2 is end of life and I really want to use it (for it's GPU) at some point of the project. This project will use Docker to containerize as much as possible.

