#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, TwistStamped

class StylusToServoController(Node):
    def __init__(self):
        super().__init__('stylus_to_servo_controller')

        # Constants
        # MoveIt Servo usually listens to delta_twist_cmds
        self.SERVO_TOPIC = "/servo_node/delta_twist_cmds"
        self.PLANNING_FRAME_ID = "base_link" # Ensure this matches your MoveIt config

        # Publishers
        self.servo_pub = self.create_publisher(TwistStamped, self.SERVO_TOPIC, 10)

        # Subscriber
        self.velocity_sub = self.create_subscription(
            Twist, 
            "/stylus_velocity", 
            self.velocity_callback, 
            10
        )

        self.get_logger().info(f"Haptic Controller Node Started. Publishing to {self.SERVO_TOPIC}")

    def velocity_callback(self, msg: Twist):
        """
        Converts the incoming Twist velocity into a TwistStamped message
        required by MoveIt Servo.
        """
        # Create the stamped message
        servo_msg = TwistStamped()
        
        # Add Header
        servo_msg.header.stamp = self.get_clock().now().to_msg()
        servo_msg.header.frame_id = self.PLANNING_FRAME_ID

        # Map Linear Velocities
        servo_msg.twist.linear.x = msg.linear.x
        servo_msg.twist.linear.y = msg.linear.y
        servo_msg.twist.linear.z = msg.linear.z

        # Map Angular Velocities (currently 0.0 from your previous node)
        servo_msg.twist.angular.x = msg.angular.x
        servo_msg.twist.angular.y = msg.angular.y
        servo_msg.twist.angular.z = msg.angular.z

        # Publish to MoveIt Servo
        self.servo_pub.publish(servo_msg)

def main(args=None):
    rclpy.init(args=args)
    node = StylusToServoController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down controller.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()



# import rclpy
# from rclpy.node import Node
# from geometry_msgs.msg import Twist, TwistStamped

# class StylusToServoController(Node):
#     def __init__(self):
#         super().__init__('stylus_to_servo_controller')

#         # 1. UPDATED TOPIC NAMES
#         # Matches the 'servo_node' name + relative topic in YAML
#         self.SERVO_TOPIC = "/servo_node/delta_twist_cmds"
        
#         # Matches the namespace we added in the launch file
#         self.TOUCH_VELOCITY_TOPIC = "/touch/stylus_velocity" 
        
#         self.PLANNING_FRAME_ID = "base_link"

#         # Publishers
#         self.servo_pub = self.create_publisher(TwistStamped, self.SERVO_TOPIC, 10)

#         # 2. UPDATED SUBSCRIBER
#         self.velocity_sub = self.create_subscription(
#             Twist, 
#             self.TOUCH_VELOCITY_TOPIC, 
#             self.velocity_callback, 
#             10
#         )

#         self.get_logger().info(f"Haptic Controller Node Started.")
#         self.get_logger().info(f"Subscribing to: {self.TOUCH_VELOCITY_TOPIC}")
#         self.get_logger().info(f"Publishing to: {self.SERVO_TOPIC}")

#     def velocity_callback(self, msg: Twist):
#         # Create the stamped message
#         servo_msg = TwistStamped()
        
#         # Use the node clock (which respects use_sim_time)
#         servo_msg.header.stamp = self.get_clock().now().to_msg()
#         servo_msg.header.frame_id = self.PLANNING_FRAME_ID

#         # Map Linear Velocities
#         servo_msg.twist.linear.x = msg.linear.x
#         servo_msg.twist.linear.y = msg.linear.y
#         servo_msg.twist.linear.z = msg.linear.z

#         # Map Angular Velocities
#         servo_msg.twist.angular.x = msg.angular.x
#         servo_msg.twist.angular.y = msg.angular.y
#         servo_msg.twist.angular.z = msg.angular.z

#         # Publish to MoveIt Servo
#         self.servo_pub.publish(servo_msg)

# def main(args=None):
#     rclpy.init(args=args)
#     node = StylusToServoController()
    
#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         node.get_logger().info("Shutting down controller.")
#     finally:
#         node.destroy_node()
#         rclpy.shutdown()

# if __name__ == '__main__':
#     main()