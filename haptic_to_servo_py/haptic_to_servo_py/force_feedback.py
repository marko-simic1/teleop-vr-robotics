#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from std_msgs.msg import Float64
from geometry_msgs.msg import Vector3, Point
from omni_msgs.msg import OmniFeedback, OmniButtonEvent

# TF2 imports for ROS 2
import tf2_ros
import tf2_geometry_msgs 
from geometry_msgs.msg import Vector3Stamped

class ForceFeedbackNode(Node):
    def __init__(self):
        super().__init__('force_feedback_node')

        # Parameters
        self.declare_parameter('robot_base_frame', 'world')
        self.declare_parameter('robot_ee_frame', 'tool_frame')
        self.robot_base_frame = self.get_parameter('robot_base_frame').value
        self.robot_ee_frame = self.get_parameter('robot_ee_frame').value

        # TF2 Buffer and Listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # State Variables
        self.apply_force = False
        self.current_ee_position = Vector3(x=0.0, y=0.0, z=0.0)
        
        # Calibration Variables
        self.is_calibrated = False
        self.calibration_samples = 500
        self.readings = []
        self.bias = 0.0

        # Subscriptions
        # Assuming the "general force_sensor" is a Float64 representing magnitude
        self.force_sub = self.create_subscription(Float64, '/force_sensor', self.force_callback, 10)
        self.button_sub = self.create_subscription(OmniButtonEvent, '/touch/button', self.button_callback, 10)

        # Publisher
        self.feedback_pub = self.create_publisher(OmniFeedback, '/touch/force_feedback', 10)

        # Timer for EE Pose Lookup (100Hz)
        self.timer = self.create_timer(0.01, self.update_ee_pose)

        self.get_logger().info(f"Node started. Target Frame: {self.robot_base_frame}")

    def update_ee_pose(self):
        """Updates the current position of the EE in the base frame."""
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                self.robot_base_frame,
                self.robot_ee_frame,
                now)
            
            self.current_ee_position.x = trans.transform.translation.x
            self.current_ee_position.y = trans.transform.translation.y
            self.current_ee_position.z = trans.transform.translation.z
        except Exception as e:
            # Throttle logging in ROS 2 isn't as simple as ROS 1, but we can catch errors
            pass

    def force_callback(self, msg):
        raw_force_value = msg.data

        # --- Calibration Logic ---
        if not self.is_calibrated:
            self.readings.append(raw_force_value)
            if len(self.readings) >= self.calibration_samples:
                self.bias = sum(self.readings) / self.calibration_samples
                self.is_calibrated = True
                self.get_logger().info(f"Calibrated! Bias: {self.bias:.3f}")
            return

        # Apply Bias
        net_force_magnitude = raw_force_value - self.bias

        # --- Directional Logic ---
        # We assume the force is applied "straight" into the manipulator. 
        # Here we represent it as a vector in the End Effector frame (e.g., along Z)
        force_ee_frame = Vector3Stamped()
        force_ee_frame.header.stamp = self.get_clock().now().to_msg()
        force_ee_frame.header.frame_id = self.robot_ee_frame
        force_ee_frame.vector.x = 0.0
        force_ee_frame.vector.y = 0.0
        force_ee_frame.vector.z = net_force_magnitude  # Straight into the tool

        try:
            # Transform the vector to the world/base frame
            transform = self.tf_buffer.lookup_transform(
                self.robot_base_frame,
                self.robot_ee_frame,
                rclpy.time.Time())
            
            # Using tf2_geometry_msgs to do the math
            transformed_force = tf2_geometry_msgs.do_transform_vector3(force_ee_frame, transform)
            
            # Prepare feedback
            feedback = OmniFeedback()
            feedback.position = self.current_ee_position
            
            if self.apply_force:
                feedback.force = transformed_force.vector
            else:
                feedback.force = Vector3(x=0.0, y=0.0, z=0.0)

            self.feedback_pub.publish(feedback)

        except Exception as e:
            self.get_logger().warn(f"TF Transform failed: {e}")

    def button_callback(self, msg):
        # ROS 2 messages use standard integer types
        if msg.grey_button == 1:
            self.apply_force = True
        else:
            self.apply_force = False

def main(args=None):
    rclpy.init(args=args)
    node = ForceFeedbackNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()