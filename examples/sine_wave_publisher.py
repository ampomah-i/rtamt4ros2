#!/usr/bin/env python3
"""Publish a deterministic signal for the rtamt4ros2 example launch file."""

import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64


class SineWavePublisher(Node):
    def __init__(self):
        super().__init__("sine_wave_source")
        self.publisher = self.create_publisher(Float64, "x", 10)
        self.elapsed = 0.0
        self.period = 0.1
        self.timer = self.create_timer(self.period, self.publish_sample)

    def publish_sample(self):
        # Range [0, 1] deliberately crosses the monitor's x > 0.5 threshold.
        value = 0.5 + 0.5 * math.sin(self.elapsed)
        self.publisher.publish(Float64(data=value))
        self.elapsed += self.period


def main(args=None):
    rclpy.init(args=args)
    node = SineWavePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
