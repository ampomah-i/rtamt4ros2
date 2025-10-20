#!/usr/bin/env python3
import ast
from typing import Dict, List, Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from std_msgs.msg import Float32
from builtin_interfaces.msg import Time as TimeMsg
from rtamt4ros2.msg import Robustness

import rtamt


class StlMonitorNode(Node):
    def __init__(self):
        super().__init__('stl_monitor_node')

        # Parameters
        self.declare_parameter('formula', 'out = always[0,5](x > 0.5)')
        self.declare_parameter('vars', ['x'])
        self.declare_parameter('sampling_period', 0.1)

        formula = self.get_parameter('formula').value
        vars_ = self.get_parameter('vars').value
        sp = float(self.get_parameter('sampling_period').value)

        # Create RTAMT discrete-time spec and pastify for online causal updates
        spec = rtamt.StlDiscreteTimeSpecification()
        for v in vars_:
            spec.declare_var(v, 'float')
        spec.declare_var('out', 'float')
        spec.spec = formula
        spec.parse()
        spec.pastify()
        spec.set_sampling_period(sp, 's')

        self.spec = spec
        self.vars: Dict[str, float] = {v: 0.0 for v in vars_}
        self.tick = 0

        # QoS: match PX4-like best-effort style for light telemetry
        qos_sub = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        qos_pub = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            depth=1
        )

        # Subscriptions to each variable name on topic f"/{name}"
        for v in vars_:
            self.create_subscription(Float32, f'/{v}', self._mk_cb(v), qos_sub)

        # Publisher for robustness
        self.pub = self.create_publisher(Robustness, '/stl_monitor/output', qos_pub)

        # Timer for periodic updates
        self.timer = self.create_timer(sp, self._on_timer)
        self.get_logger().info(f"Started STL monitor with formula: {formula}")
        self.get_logger().info(f"Variables: {vars_} | sampling_period={sp}s")

    def _mk_cb(self, name: str):
        def _cb(msg: Float32):
            self.vars[name] = float(msg.data)
        return _cb

    def _on_timer(self):
        try:
            data = list(self.vars.items())
            r = float(self.spec.update(self.tick, data))
            msg = Robustness()
            msg.stamp = self.get_clock().now().to_msg()
            msg.robustness = r
            msg.satisfied = (r >= 0.0)
            msg.formula = self.spec.spec
            self.pub.publish(msg)
            self.tick += 1
        except Exception as e:
            self.get_logger().error(f"RTAMT update failed: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = StlMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
