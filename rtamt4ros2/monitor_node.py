#!/usr/bin/python3
"""ROS 2 node for online Signal Temporal Logic monitoring."""

from functools import partial
from math import isfinite
from pathlib import Path
from typing import Dict

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

from rosidl_runtime_py.utilities import get_message
from rtamt4ros2.msg import Robustness
from rtamt4ros2.monitor import (
    MonitorConfigurationError,
    OnlineStlMonitor,
    extract_numeric_field,
)


class StlMonitorNode(Node):
    def __init__(self):
        super().__init__('stl_monitor_node')

        # Parameters
        self.declare_parameter("formula", "out = always[0,5](x > 0.5)")
        self.declare_parameter("formula_file", "")
        self.declare_parameter("vars", ["x"])
        self.declare_parameter("input_topics", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("input_type", "std_msgs/msg/Float64")
        self.declare_parameter("input_types", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("input_fields", Parameter.Type.STRING_ARRAY)
        self.declare_parameter("output_topic", "stl_monitor/output")
        self.declare_parameter("sampling_period", 0.1)
        self.declare_parameter("sampling_unit", "s")
        self.declare_parameter("require_all_inputs", True)
        self.declare_parameter("qos_reliability", "best_effort")
        self.declare_parameter("qos_depth", 10)

        def string_array_parameter(name):
            fallback = Parameter(name, Parameter.Type.STRING_ARRAY, [])
            return list(self.get_parameter_or(name, fallback).value)

        formula = str(self.get_parameter("formula").value)
        formula_file = str(self.get_parameter("formula_file").value)
        variables = list(self.get_parameter("vars").value)
        topics = string_array_parameter("input_topics")
        default_input_type = str(self.get_parameter("input_type").value)
        input_types = string_array_parameter("input_types")
        input_fields = string_array_parameter("input_fields")
        output_topic = str(self.get_parameter("output_topic").value)
        sampling_period = float(self.get_parameter("sampling_period").value)
        sampling_unit = str(self.get_parameter("sampling_unit").value)
        self.require_all_inputs = bool(self.get_parameter("require_all_inputs").value)
        reliability_name = str(self.get_parameter("qos_reliability").value)
        qos_depth = int(self.get_parameter("qos_depth").value)

        if formula_file:
            path = Path(formula_file).expanduser()
            try:
                formula = path.read_text(encoding="utf-8").strip()
            except OSError as exc:
                raise MonitorConfigurationError(
                    f"cannot read formula_file {formula_file!r}: {exc}"
                ) from exc

        if not topics:
            topics = variables
        if not input_types:
            input_types = [default_input_type] * len(variables)
        if not input_fields:
            input_fields = ["data"] * len(variables)
        for parameter_name, values in (
            ("input_topics", topics),
            ("input_types", input_types),
            ("input_fields", input_fields),
        ):
            if len(values) != len(variables):
                raise MonitorConfigurationError(
                    f"{parameter_name} must be empty or have exactly one entry per variable"
                )
        if len(topics) != len(set(topics)):
            raise MonitorConfigurationError("input_topics must be unique")
        if qos_depth <= 0:
            raise MonitorConfigurationError("qos_depth must be positive")
        if reliability_name not in {"best_effort", "reliable"}:
            raise MonitorConfigurationError(
                "qos_reliability must be 'best_effort' or 'reliable'"
            )
        message_classes = []
        for input_type in input_types:
            try:
                message_classes.append(get_message(input_type))
            except (AttributeError, ModuleNotFoundError, ValueError) as exc:
                raise MonitorConfigurationError(
                    f"invalid input type {input_type!r}: {exc}"
                ) from exc

        self.monitor = OnlineStlMonitor(
            formula, variables, sampling_period, sampling_unit
        )
        self.values: Dict[str, float] = {}

        reliability = (
            QoSReliabilityPolicy.BEST_EFFORT
            if reliability_name == "best_effort"
            else QoSReliabilityPolicy.RELIABLE
        )
        qos_sub = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=reliability,
            depth=qos_depth,
        )
        qos_pub = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=reliability,
            depth=1,
        )

        self._input_subscriptions = [
            self.create_subscription(
                message_class,
                topic,
                partial(self._on_input, variable, field),
                qos_sub,
            )
            for variable, topic, message_class, field in zip(
                variables, topics, message_classes, input_fields
            )
        ]

        self.pub = self.create_publisher(Robustness, output_topic, qos_pub)

        unit_seconds = {"s": 1.0, "ms": 1e-3, "us": 1e-6, "ns": 1e-9}
        timer_period = sampling_period * unit_seconds[sampling_unit]
        self.timer = self.create_timer(timer_period, self._on_timer)
        mappings = ", ".join(
            f"{variable}={topic} ({input_type}:{field})"
            for variable, topic, input_type, field in zip(
                variables, topics, input_types, input_fields
            )
        )
        self.get_logger().info(f"Started STL monitor: {formula}")
        self.get_logger().info(
            f"Inputs: {mappings}; sampling_period={sampling_period}{sampling_unit}"
        )

    def _on_input(self, name, field_path, msg):
        try:
            value = extract_numeric_field(msg, field_path)
        except (AttributeError, TypeError, ValueError) as exc:
            self.get_logger().error(
                f"Cannot extract numeric field {field_path!r} for {name}: {exc}"
            )
            return
        if not isfinite(value):
            self.get_logger().warning(f"Ignoring non-finite sample for {name}")
            return
        self.values[name] = value

    def _on_timer(self):
        missing = set(self.monitor.variables).difference(self.values)
        if missing and self.require_all_inputs:
            return
        try:
            sample = {name: self.values.get(name, 0.0) for name in self.monitor.variables}
            robustness = self.monitor.update(sample)
            msg = Robustness()
            msg.stamp = self.get_clock().now().to_msg()
            msg.robustness = robustness
            msg.satisfied = robustness >= 0.0
            msg.formula = self.monitor.formula
            self.pub.publish(msg)
        except Exception as exc:
            self.get_logger().error(f"RTAMT update failed: {exc}")


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = StlMonitorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except MonitorConfigurationError as exc:
        rclpy.logging.get_logger("stl_monitor_node").fatal(str(exc))
        raise
    finally:
        try:
            if node is not None:
                node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
