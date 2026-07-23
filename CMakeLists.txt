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
  PATTERN "__pycache__" EXCLUDE
)

# RTAMT 0.3.5 pins ANTLR 4.7. Install both vendored, pure-Python modules into
# the ROS prefix to keep binary packages reproducible and independent of PyPI.
install(
  DIRECTORY vendor/rtamt vendor/antlr4
  DESTINATION "${PYTHON_INSTALL_DIR}"
  FILES_MATCHING PATTERN "*.py"
  PATTERN "__pycache__" EXCLUDE
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
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)

install(FILES
  LICENSE
  vendor/LICENSE.rtamt
  vendor/LICENSE.antlr4
  DESTINATION share/${PROJECT_NAME}/licenses
)

if(BUILD_TESTING)
  find_package(ament_cmake_flake8 REQUIRED)
  find_package(ament_cmake_pep257 REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_xmllint REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  ament_add_pytest_test(test_monitor tests/test_monitor.py)
  ament_add_pytest_test(test_launch_file tests/test_launch_file.py)
  add_launch_test(tests/test_monitor_launch.py TIMEOUT 45)

  # Keep third-party vendored sources out of first-party style checks.
  ament_flake8(
    CONFIG_FILE tests/ament_flake8.ini
    rtamt4ros2 examples launch tests
  )
  ament_pep257(rtamt4ros2 examples launch tests)
  ament_xmllint(package.xml)
endif()

ament_export_dependencies(rosidl_default_runtime)
ament_package()
