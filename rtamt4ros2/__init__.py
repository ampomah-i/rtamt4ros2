__all__ = ["monitor_node"]
"""ROS 2 integration for RTAMT."""

# During source-space development the generated ``rtamt4ros2.msg`` package is
# in the build/install space. Let Python combine both package locations.
from pkgutil import extend_path

__path__ = extend_path(__path__, __name__)
