#!/usr/bin/env python3
import ast
import os
from typing import Dict, Any, Callable, Optional
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from std_msgs.msg import Float32, Float64, Bool, Int32
from geometry_msgs.msg import Point, Pose, Twist, Vector3
from rtamt4ros2.msg import Robustness
import rtamt


class StlMonitorNode(Node):
    """
    Runtime STL monitor for ROS 2 with support for:
    - File-based or inline specifications
    - Flexible topic mapping (var name ≠ topic name)
    - Multiple message types (Float32, Point, Pose, etc.)
    - Dynamic frequency from spec
    """
    
    # Supported message types
    MESSAGE_TYPES = {
        'Float32': Float32,
        'Float64': Float64,
        'Bool': Bool,
        'Int32': Int32,
        'Point': Point,
        'Pose': Pose,
        'Twist': Twist,
        'Vector3': Vector3,
    }
    
    def __init__(self):
        # Initialize with default name (may be overridden)
        super().__init__('stl_monitor_node')
        
        # ============= PARAMETERS =============
        # Specification source (file or inline)
        self.declare_parameter('formula_file', '')
        self.declare_parameter('formula', 'out = always[0,5](x > 0.5)')
        
        # Variables and timing
        self.declare_parameter('vars', ['x'])
        self.declare_parameter('sampling_period', 0.1)
        self.declare_parameter('sampling_unit', 's')
        
        # Topic mapping: {var_name: topic_name}
        self.declare_parameter('topic_mapping', {})
        
        # Message types: {var_name: msg_type_string}
        self.declare_parameter('var_types', {})
        
        # Field extraction for complex messages: {var_name: field_path}
        # e.g., {'x': 'pose.position.x'} for Pose messages
        self.declare_parameter('var_fields', {})
        
        # Monitoring options
        self.declare_parameter('require_all_vars', True)
        self.declare_parameter('publish_period', True)  # Publish every period even if no new data
        
        # QoS options
        self.declare_parameter('qos_reliability', 'best_effort')
        self.declare_parameter('qos_depth', 10)
        
        # ============= LOAD PARAMETERS =============
        formula_file = self.get_parameter('formula_file').value
        formula_inline = self.get_parameter('formula').value
        vars_param = self.get_parameter('vars').value
        sp = float(self.get_parameter('sampling_period').value)
        unit = self.get_parameter('sampling_unit').value
        
        # Parse complex parameters
        vars_ = self._parse_param(vars_param, list)
        topic_mapping = self._parse_param(self.get_parameter('topic_mapping').value, dict)
        var_types = self._parse_param(self.get_parameter('var_types').value, dict)
        var_fields = self._parse_param(self.get_parameter('var_fields').value, dict)
        
        self.require_all = self.get_parameter('require_all_vars').value
        self.publish_period = self.get_parameter('publish_period').value
        
        # ============= LOAD FORMULA =============
        if formula_file:
            if not os.path.exists(formula_file):
                raise FileNotFoundError(f"Formula file not found: {formula_file}")
            
            with open(formula_file, 'r') as f:
                formula = f.read().strip()
            self.get_logger().info(f"📄 Loaded formula from: {formula_file}")
        elif formula_inline:
            formula = formula_inline
            self.get_logger().info(f"📝 Using inline formula")
        else:
            raise ValueError("Must provide either 'formula_file' or 'formula' parameter")
        
        # ============= INITIALIZE RTAMT =============
        try:
            spec = rtamt.StlDiscreteTimeSpecification()
            
            # Declare variables
            for v in vars_:
                spec.declare_var(v, 'float')
            spec.declare_var('out', 'float')
            
            # Parse and pastify
            spec.spec = formula
            spec.parse()
            spec.pastify()
            spec.set_sampling_period(sp, unit)
            
            self.spec = spec
            self.sampling_period = sp
            self.sampling_unit = unit
            self.frequency = spec.get_sampling_frequency()
            
            self.get_logger().info(f"✓ Formula parsed: {formula}")
            self.get_logger().info(f"⏱  Sampling: {sp} {unit} ({self.frequency} Hz)")
            
        except rtamt.STLParseException as e:
            self.get_logger().error(f"✗ STL Parse Exception: {e}")
            raise
        except Exception as e:
            self.get_logger().error(f"✗ RTAMT initialization failed: {e}")
            raise
        
        # ============= STATE TRACKING =============
        self.vars: Dict[str, float] = {v: 0.0 for v in vars_}
        self.vars_received: Dict[str, bool] = {v: False for v in vars_}
        self.vars_timestamps: Dict[str, float] = {v: 0.0 for v in vars_}
        self.tick = 0
        self.last_update_time: Optional[rclpy.time.Time] = None
        
        # ============= QoS CONFIGURATION =============
        qos_reliability_param = self.get_parameter('qos_reliability').value
        qos_depth = self.get_parameter('qos_depth').value
        
        reliability = (QoSReliabilityPolicy.BEST_EFFORT 
                      if qos_reliability_param == 'best_effort' 
                      else QoSReliabilityPolicy.RELIABLE)
        
        qos_sub = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            reliability=reliability,
            depth=qos_depth
        )
        
        qos_pub = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
            depth=1
        )
        
        # ============= SUBSCRIPTIONS =============
        self.subscribers = []
        for v in vars_:
            # Determine topic name
            topic = topic_mapping.get(v, f'/{v}')
            
            # Determine message type
            msg_type_str = var_types.get(v, 'Float32')
            if msg_type_str not in self.MESSAGE_TYPES:
                raise ValueError(f"Unsupported message type: {msg_type_str}")
            msg_type = self.MESSAGE_TYPES[msg_type_str]
            
            # Determine field extraction
            field_path = var_fields.get(v, None)
            
            # Create callback
            callback = self._create_callback(v, msg_type_str, field_path)
            
            # Subscribe
            sub = self.create_subscription(msg_type, topic, callback, qos_sub)
            self.subscribers.append(sub)
            
            self.get_logger().info(
                f"📡 Variable '{v}' ← topic '{topic}' "
                f"(type: {msg_type_str}{f', field: {field_path}' if field_path else ''})"
            )
        
        # ============= PUBLISHER =============
        self.pub = self.create_publisher(Robustness, '/stl_monitor/output', qos_pub)
        
        # ============= TIMER =============
        timer_period = 1.0 / self.frequency  # Convert Hz to seconds
        self.timer = self.create_timer(timer_period, self._on_timer)
        
        self.get_logger().info(f"🚀 STL Monitor started!")
    
    def _parse_param(self, param_value: Any, expected_type: type) -> Any:
        """Parse parameters that might be strings or actual types"""
        if isinstance(param_value, str) and param_value:
            try:
                return ast.literal_eval(param_value)
            except (ValueError, SyntaxError) as e:
                self.get_logger().warn(f"Failed to parse '{param_value}': {e}")
                return expected_type()
        elif isinstance(param_value, expected_type):
            return param_value
        else:
            return expected_type()
    
    def _create_callback(self, var_name: str, msg_type: str, field_path: Optional[str]) -> Callable:
        """Create a callback function for a specific variable and message type"""
        
        def extract_value(msg):
            """Extract float value from message based on type and field path"""
            if msg_type in ['Float32', 'Float64']:
                return float(msg.data)
            elif msg_type == 'Bool':
                return float(msg.data)
            elif msg_type == 'Int32':
                return float(msg.data)
            elif msg_type == 'Point':
                if field_path:
                    return self._get_nested_attr(msg, field_path)
                return float(msg.x)  # Default to x
            elif msg_type == 'Pose':
                if field_path:
                    return self._get_nested_attr(msg, field_path)
                return float(msg.position.x)  # Default
            elif msg_type == 'Twist':
                if field_path:
                    return self._get_nested_attr(msg, field_path)
                return float(msg.linear.x)  # Default
            elif msg_type == 'Vector3':
                if field_path:
                    return self._get_nested_attr(msg, field_path)
                return float(msg.x)  # Default
            else:
                raise ValueError(f"Unsupported message type: {msg_type}")
        
        def callback(msg):
            try:
                value = extract_value(msg)
                self.vars[var_name] = value
                self.vars_received[var_name] = True
                self.vars_timestamps[var_name] = self.get_clock().now().nanoseconds / 1e9
                
                # Log first message
                if not self.vars_received[var_name]:
                    self.get_logger().info(f"✓ First message received for '{var_name}'", once=True)
                    
            except Exception as e:
                self.get_logger().error(f"Error extracting value from {var_name}: {e}")
        
        return callback
    
    def _get_nested_attr(self, obj: Any, path: str) -> float:
        """Get nested attribute using dot notation (e.g., 'pose.position.x')"""
        attrs = path.split('.')
        for attr in attrs:
            obj = getattr(obj, attr)
        return float(obj)
    
    def _on_timer(self):
        """Periodic evaluation of STL specification"""
        
        # Check if we have all required data
        if self.require_all and not all(self.vars_received.values()):
            missing = [k for k, v in self.vars_received.items() if not v]
            self.get_logger().warn(
                f"⏳ Waiting for variables: {missing}", 
                throttle_duration_sec=5.0
            )
            return
        
        # Check for timing jitter
        current_time = self.get_clock().now()
        if self.last_update_time is not None:
            expected_dt = 1.0 / self.frequency
            actual_dt = (current_time - self.last_update_time).nanoseconds / 1e9
            if abs(actual_dt - expected_dt) > 0.05 * expected_dt:
                self.get_logger().warn(
                    f"⚠ Timer jitter: {actual_dt:.3f}s (expected {expected_dt:.3f}s)",
                    throttle_duration_sec=10.0
                )
        self.last_update_time = current_time
        
        # Evaluate specification
        try:
            # Prepare data in RTAMT format: list of (name, value) tuples
            data = list(self.vars.items())
            
            # Update specification
            robustness = float(self.spec.update(self.tick, data))
            
            # Create and publish message
            msg = Robustness()
            msg.stamp = current_time.to_msg()
            msg.robustness = robustness
            msg.satisfied = (robustness >= 0.0)
            msg.formula = self.spec.spec
            
            self.pub.publish(msg)
            
            # Log result
            self.get_logger().info(
                f"Tick {self.tick}: ρ={robustness:.3f}, "
                f"{'✓ SAT' if robustness >= 0.0 else '✗ UNSAT'}",
                throttle_duration_sec=1.0
            )
            
            self.tick += 1
            
        except rtamt.RTAMTException as e:
            self.get_logger().error(f"RTAMT evaluation error: {e}")
        except Exception as e:
            self.get_logger().fatal(f"Unexpected error: {e}")
            raise


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = StlMonitorNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Fatal error: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
