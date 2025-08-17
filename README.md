# ROS2 implementation of Freenove Big Hexapod Robot kit for Raspberry Pi (FNK0052)

### Thank you to Freenova
Firstly, I'd like to thank the ppl at Freenove for being awesome and making applications like this available open source, no strings attached. This is the way. 🙏

<img src='assets/images/icon.png' width='50%'/>

## ROS2 Development

### Hardware
The main hardware components of this project:
- Freenove FNK0052 kit
- Raspberry Pi 5
- Windows 11 laptop with WSL2
- Nvidia Jetson TX2
- Intel Realsense Depth Sensor
- 4S ~14v Battery

### Project Goals
This project aims to accomplish the following goals.
- Implement a ROS distributed systems to control the Hexapod Robot
  - windows_command: A pyqt5 application on a Windows PC (WSL2) to command the robot. 
  - pi_control: A Raspberry Pi 5 receives commands to actuate motors. 
  - jetson_vision: An nvidia Jetson TX2 streams Realsense depth data and runs AI models onboard .
  - robot_interfaces: all the service, msg, etc. type definitions for communication between nodes.
- Rewrite the Freenove opensource implementation into ROS compatible packages. 
- Implement custom motion (joint motion) (frame of resolution motion)
- Implement a Simulation environment (rvid? something else?) to train the robot to climb stairs.
- and more! 

### Starting Out
For assembly - the best place to start is by following Freenova's kit Tutorial.pdf. This is available in the branch: <a href="[link](https://github.com/ogordillo/ROS2_Conversion_Freenove_Big_Hexapod_Robot_Raspberry_Pi/tree/original)">original</a> or by following the steps from Freenove project <a href="[link](https://github.com/Freenove/Freenove_Big_Hexapod_Robot_Kit_for_Raspberry_Pi)">Freenove_Big_Hexapod_Robot_Kit_for_Raspberry_Pi</a>

Once the robot is assembled and operational, there are README.md files in the respective component folders to guide you through setting up the docker containers and nuances between the platforms.

- src
  - pi_control
    - README.md
  - windows_command
    - README.md
  - jetson_vision
    - README.md

### Github Page
There is a github page where I post about updates on the project. This blog includes pictures, videos and talks about things that worked or didn't worked. 

### Distributed Setup
This project uses CycloneDDS RMW (Middleware) for p2p discovery.

### Docker Containers
- docker
  - docker.pi
  - docker.jetson (jetson)
  - docker.wsl (windows)
    
### Note about using EOL ROS2 Foxy
I am personally working towards a ROS2 Foxy build because my Jetson TX2 is end of life and I really want to use it (for edge computing). This project will use Docker to containerize as much as possible to more easily pivot if needed. 

### Contributing
Contributions are welcome! If you have an idea, find a bug, or want to add a new feature, please open an issue or submit a pull request. 

