"""End-to-end launch test for the installed ROS 2 monitor node."""

import time
import unittest

import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
import pytest
import rclpy
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy
from rtamt4ros2.msg import Robustness
from std_msgs.msg import Float64


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    """Launch a monitor configured with a deterministic atomic predicate."""
    monitor = launch_ros.actions.Node(
        package="rtamt4ros2",
        executable="stl_monitor_node",
        name="stl_monitor_integration_test",
        output="screen",
        parameters=[
            {
                "formula": "out = x > 0.5",
                "vars": ["x"],
                "input_topics": ["/rtamt4ros2_test/x"],
                "output_topic": "/rtamt4ros2_test/robustness",
                "sampling_period": 0.02,
                "qos_reliability": "best_effort",
            }
        ],
    )
    return launch.LaunchDescription(
        [monitor, launch_testing.actions.ReadyToTest()]
    )


class TestMonitorNode(unittest.TestCase):
    """Exercise the monitor through its public ROS topics."""

    def setUp(self):
        """Create a test publisher and output subscription."""
        rclpy.init()
        self.node = rclpy.create_node("rtamt4ros2_integration_client")
        self.publisher = self.node.create_publisher(
            Float64, "/rtamt4ros2_test/x", 10
        )
        output_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=10,
        )
        self.outputs = []
        self.subscription = self.node.create_subscription(
            Robustness,
            "/rtamt4ros2_test/robustness",
            self.outputs.append,
            output_qos,
        )

    def tearDown(self):
        """Destroy the test node and its ROS context."""
        self.node.destroy_node()
        rclpy.shutdown()

    def test_publishes_expected_robustness(self):
        """A positive input produces a satisfied message with robustness 0.75."""
        deadline = time.monotonic() + 15.0
        sample = Float64(data=1.25)

        while time.monotonic() < deadline and not self.outputs:
            self.publisher.publish(sample)
            rclpy.spin_once(self.node, timeout_sec=0.05)

        self.assertTrue(self.outputs, "monitor did not publish a robustness result")
        result = self.outputs[-1]
        self.assertAlmostEqual(result.robustness, 0.75)
        self.assertTrue(result.satisfied)
        self.assertEqual(result.formula, "out = x > 0.5")
