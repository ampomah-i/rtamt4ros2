cmake_minimum_required(VERSION 3.10)
project(rtamt4ros2)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Robustness.msg"
  DEPENDENCIES builtin_interfaces
)

# rosidl_generator_py creates the generated rtamt4ros2 Python package. Install
# our modules into that same package without creating a duplicate CMake target.
ament_get_python_install_dir(PYTHON_INSTALL_DIR)
install(
  DIRECTORY rtamt4ros2/
  DESTINATION "${PYTHON_INSTALL_DIR}/${PROJECT_NAME}"
  FILES_MATCHING PATTERN "*.py"
)

install(PROGRAMS
  rtamt4ros2/monitor_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME stl_monitor_node
)

install(PROGRAMS
  examples/sine_wave_publisher.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME sine_wave_publisher
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_monitor tests/test_monitor.py)
  ament_add_pytest_test(test_launch_file tests/test_launch_file.py)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
