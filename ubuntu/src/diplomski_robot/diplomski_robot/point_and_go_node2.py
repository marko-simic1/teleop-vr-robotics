#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry


def clamp(value, min_val, max_val):
    """Ograničava vrijednost da ne pređe min/max granice."""
    return max(min_val, min(value, max_val))

def yaw_from_quaternion(q):
    """Pretvara orijentaciju (kvaternion) u kut zakretanja (yaw)."""
    t3 = +2.0 * (q.w * q.z + q.x * q.y)
    t4 = +1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(t3, t4)

class PointAndGo(Node):

    def __init__(self):
        super().__init__("point_and_go")

        self.goal_topic = "/goal_point"
        self.odom_topic = "/odom"
        self.cmd_topic = "/cmd_vel_nav" 
        self.k_lin = 0.8          
        self.k_ang = 1.8          
        self.max_lin = 0.6        
        self.max_ang = 1.5        
        self.goal_tolerance = 0.15 
        
        self.turn_in_place_angle = 0.5 

        self.goal = None
        self.last_odom = None

        self.create_subscription(Point, self.goal_topic, self.on_goal, 10)
        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 20)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 20)
        
        self.create_timer(0.05, self.control_loop)

        self.get_logger().info(f"Listening goal: {self.goal_topic}, odom: {self.odom_topic}, publishing: {self.cmd_topic}")

    def on_goal(self, msg: Point):
        self.goal = (msg.x, msg.y, msg.z)
        self.get_logger().info(f"New goal: x={msg.x:.2f}, y={msg.y:.2f}")

    def on_odom(self, msg: Odometry):
        self.last_odom = msg

    def stop_robot(self):
        t = Twist()
        self.cmd_pub.publish(t)

    def control_loop(self):
        if self.goal is None or self.last_odom is None:
            return

        od = self.last_odom
        x = od.pose.pose.position.x
        y = od.pose.pose.position.y
        
        yaw = yaw_from_quaternion(od.pose.pose.orientation)

        gx, gy, _ = self.goal

        dx = gx - x
        dy = gy - y
        dist = math.hypot(dx, dy)

        if dist < self.goal_tolerance:
            self.stop_robot()
            self.get_logger().info("Goal reached -> stop.")
            self.goal = None
            return

        desired_yaw = math.atan2(dy, dx)
        yaw_err = desired_yaw - yaw
        yaw_err = (yaw_err + math.pi) % (2.0 * math.pi) - math.pi

        cmd = Twist()

        if abs(yaw_err) > self.turn_in_place_angle:
            cmd.linear.x = 0.0
            cmd.angular.z = clamp(self.k_ang * yaw_err, -self.max_ang, self.max_ang)
        else:
            cmd.linear.x = clamp(self.k_lin * dist, 0.0, self.max_lin)
            cmd.angular.z = clamp(self.k_ang * yaw_err, -self.max_ang, self.max_ang)

        self.cmd_pub.publish(cmd)

def main():
    rclpy.init()
    node = PointAndGo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.stop_robot()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
