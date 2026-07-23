# Contributing

Create changes against a supported ROS 2 distribution, install dependencies, and run:

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
source .venv/bin/activate
colcon build --packages-select rtamt4ros2 --symlink-install
source install/setup.bash
colcon test --packages-select rtamt4ros2
colcon test-result --verbose
```

Changes to monitoring behavior should include unit tests in `tests/test_monitor.py`.
Changes to ROS parameters or interfaces should update the example configuration and README.
