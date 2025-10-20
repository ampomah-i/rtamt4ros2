cmake_minimum_required(VERSION 3.8)
project(rtamt4ros2)

find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate the message interface
rosidl_generate_interfaces(${PROJECT_NAME}
  "rtamt4ros2/msg/Robustness.msg"
  DEPENDENCIES builtin_interfaces
)

# Ensure Python is installed via setup.py
ament_python_install_package(${PROJECT_NAME})

# Install the node script
install(PROGRAMS
  rtamt4ros2/monitor_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
