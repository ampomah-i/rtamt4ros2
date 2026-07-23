# rtamt4ros2

`rtamt4ros2` evaluates Signal Temporal Logic (STL) specifications online from
ROS 2 scalar topics. It uses [RTAMT](https://github.com/nickovic/rtamt) for
quantitative semantics and publishes both robustness and Boolean satisfaction.

The package is developed by Cornell University's Verifiable Robotics Group.
For ROS 1, see [`rtamt4ros`](https://github.com/nickovic/rtamt4ros).

## Features

- Discrete-time online monitoring of pastified, causal STL specifications
- Configurable variable-to-topic mappings
- Any installed scalar ROS message with a numeric `data` field
- No output before all required inputs have arrived
- Timestamped robustness and satisfaction output
- Unit-tested ROS-independent monitoring core
- Launch file, YAML configuration, and runnable signal-source example

## Install and build

The current RTAMT PyPI release is `0.3.5` and requires its matching ANTLR
runtime. Use the system Python associated with your ROS installation:

```bash
sudo apt update
sudo apt install ros-${ROS_DISTRO}-rclpy \
  ros-${ROS_DISTRO}-rosidl-runtime-py \
  ros-${ROS_DISTRO}-std-msgs \
  python3-pip python3-venv

mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/VerifiableRobotics/rtamt4ros2.git
cd ..
source /opt/ros/${ROS_DISTRO}/setup.bash

# Isolate RTAMT's ANTLR 4.7 dependency from other Python applications.
/usr/bin/python3 -m venv --system-site-packages .venv
source .venv/bin/activate
python -m pip install "rtamt==0.3.5"

colcon build --packages-select rtamt4ros2 --symlink-install
source install/setup.bash
```

Keep the virtual environment activated when building or running the package.
Avoid Conda or an interpreter that differs from the one used by ROS 2; binary
`rclpy` extensions are tied to that interpreter. If your user site contains
ROS 1 packages such as `py3rosmsgs`, run `export PYTHONNOUSERSITE=1` before
activating the environment so they cannot shadow ROS 2 interfaces.

## Run the example

```bash
ros2 launch rtamt4ros2 example.launch.py
```

In another terminal:

```bash
source ~/ros2_ws/install/setup.bash
ros2 topic echo /stl_monitor/output
```

The example publishes a sine wave on `/x`. The default specification,
`out = always[0,5](x > 0.5)`, alternates between satisfied and violated.

## Configure a monitor

Copy `config/monitor.yaml`, edit it, and launch it with:

```bash
ros2 launch rtamt4ros2 stl_monitor.launch.py config:=/absolute/path/to/monitor.yaml
```

Parameters:

| Parameter | Type | Default | Meaning |
|---|---|---|---|
| `formula` | string | `out = always[0,5](x > 0.5)` | Full RTAMT assignment; output must be named `out` |
| `formula_file` | string | `""` | Optional STL file; takes precedence over `formula` |
| `vars` | string array | `["x"]` | STL input variable names |
| `input_topics` | string array | `[]` | Topic for each variable, in the same order; an empty list uses each variable name |
| `input_type` | string | `std_msgs/msg/Float64` | Default input type when `input_types` is empty |
| `input_types` | string array | `[]` | Per-variable fully qualified ROS interface types |
| `input_fields` | string array | `[]` | Per-variable dotted numeric field paths; defaults to `data` |
| `output_topic` | string | `stl_monitor/output` | Robustness output topic |
| `sampling_period` | double | `0.1` | Monitor update period |
| `sampling_unit` | string | `s` | Sampling unit: `s`, `ms`, `us`, or `ns` |
| `require_all_inputs` | bool | `true` | Wait for every input; if false, missing inputs start at zero |
| `qos_reliability` | string | `best_effort` | Input/output reliability: `best_effort` or `reliable` |
| `qos_depth` | integer | `10` | Input subscription history depth |

For two signals:

```yaml
stl_monitor:
  ros__parameters:
    formula: "out = always[0,10]((speed < 2.0) or (distance > 1.5))"
    vars: ["speed", "distance"]
    input_topics: ["/vehicle/speed", "/lidar/min_distance"]
    input_types: ["std_msgs/msg/Float64", "sensor_msgs/msg/Range"]
    input_fields: ["data", "range"]
    sampling_period: 0.05
```

Structured messages are supported through dotted fields, for example
`pose.position.x` for `geometry_msgs/msg/Pose`.

## Output interface

`rtamt4ros2/msg/Robustness` contains:

```text
builtin_interfaces/Time stamp
float64 robustness
bool satisfied
string formula
```

Satisfaction is true when robustness is greater than or equal to zero.

## Test

```bash
source /opt/ros/${ROS_DISTRO}/setup.bash
source .venv/bin/activate
colcon build --packages-select rtamt4ros2 --symlink-install
source install/setup.bash
colcon test --packages-select rtamt4ros2
colcon test-result --verbose
```

## Binary-release status

The source package is structured for Bloom and includes a changelog. A ROS
binary release still requires an accepted rosdep key (or a separate ROS release)
for the third-party `rtamt` Python dependency. Until that exists, install RTAMT
from PyPI before building. After the dependency is resolvable through rosdep,
the remaining release steps are creating a release repository, running Bloom,
and submitting the generated rosdistro pull request.

## License

MIT
