#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion
from nav_msgs.msg import Odometry
from builtin_interfaces.msg import Time

def quat_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.w = math.cos(yaw * 0.5)
    q.x = 0.0
    q.y = 0.0
    q.z = math.sin(yaw * 0.5)
    return q

class MovingRobot(Node):
    def __init__(self):
        super().__init__("moving_robot")

        self.cmd_topic = "/cmd_vel"
        self.odom_topic = "/odom"

        self.x = 0.0
        self.y = 0.0
        #direction in which the robot is looking at 
        #yaw = 0 -> right
        #yaw = 90 -> up
        #yaw = 180 -> left
        #yaw = -90 -> down
        self.yaw = 0.0
        self.v = 0.0
        self.w = 0.0

        self.last_time = self.get_clock().now()

        self.create_subscription(Twist, self.cmd_topic, self.on_cmd, 20)
        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 20)
        self.create_timer(0.02, self.step) 

        self.get_logger().info(f"Simulating odom from {self.cmd_topic} -> publishing {self.odom_topic}")

    def on_cmd(self, msg: Twist):
        self.v = msg.linear.x
        self.w = msg.angular.z

    def step(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds * 1e-9
        self.last_time = now
        if dt <= 0.0:
            return

        # unicycle model
        self.yaw += self.w * dt
        self.x += self.v * math.cos(self.yaw) * dt
        self.y += self.v * math.sin(self.yaw) * dt

        odom = Odometry()
        odom.header.stamp = now.to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = quat_from_yaw(self.yaw)

        odom.twist.twist.linear.x = self.v
        odom.twist.twist.angular.z = self.w

        self.odom_pub.publish(odom)

def main():
    rclpy.init()
    node = MovingRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
