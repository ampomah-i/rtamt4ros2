# rtamt4ros2

Runtime monitoring for ROS 2 using [RTAMT](https://github.com/nickovic/rtamt).
Evaluate Signal Temporal Logic (STL) specifications **online** on ROS 2 topics and publish robustness and boolean satisfaction.

This package is developed as part of ongoing work in the **Verifiable Robotics Group at Cornell University**, focusing on safe and verifiable autonomy.

> If you are looking for the ROS 1 version, see [`rtamt4ros`](https://github.com/nickovic/rtamt4ros).  
> This repository provides a ROS 2-native monitor node built on `rclpy`.

## Features
- Online (streaming) monitoring with **pastified** STL for causal updates
- Publishes quantitative **robustness** and **boolean** truth
- Distro-agnostic: tested on Humble; designed to work across ROS 2 distros
- Simple message interface: `rtamt4ros2/Robustness`

## Quick start

### 1) Install dependencies
```bash
# System packages
sudo apt-get update
sudo apt-get install -y ros-${ROS_DISTRO}-rclpy ros-${ROS_DISTRO}-std-msgs

# Python deps
pip install rtamt
```

### 2) Build in your workspace
```bash
cd ~/ros2_ws/src
# Place this folder here: rtamt4ros2/
cd ..
colcon build --packages-select rtamt4ros2
source install/setup.bash
```

### 3) Run the monitor node
```bash
ros2 run rtamt4ros2 stl_monitor_node --ros-args   -p formula:="out = always[0,5](x > 0.5)"   -p vars:="['x']"   -p sampling_period:=0.1
```

Publish a test signal:
```bash
ros2 topic pub /x std_msgs/msg/Float32 "{data: 1.0}"
```

Observe outputs:
```bash
ros2 topic echo /stl_monitor/output
```

## Parameters
- `formula` *(string)*: Full RTAMT STL assignment, e.g. `out = always[0,5](x > 0.5)`
- `vars` *(string array)*: Variable names to subscribe to as `/VAR` topics (Float32)
- `sampling_period` *(double)*: Sampling period in seconds

## Message
`rtamt4ros2/Robustness`:
```text
builtin_interfaces/Time stamp
float64 robustness
bool   satisfied
string formula
```

## Notes
- The node uses **discrete-time** RTAMT with `pastify()` for online, causal updates.
- With inclusive bounds, `always[0,k]` spans `k+1` samples; violations are reported with a `k`-tick delay relative to the start of a violating run.

## Acknowledgments
This work was conducted as part of the **Verifiable Robotics Group at Cornell University**, advised by **Prof. Hadas Kress-Gazit**, and builds upon the original [**rtamt4ros**](https://github.com/nickovic/rtamt4ros) framework developed by **Nikolaos Nickovic**
## Maintainer
**Immanuel Ampomah Mensah** — ia324@cornell.edu

## License
MIT
