cmake_minimum_required(VERSION 3.8)
project(robot_interfaces)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate services
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SetServo.srv"
)

ament_package()