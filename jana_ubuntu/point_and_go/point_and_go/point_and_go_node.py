#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist
from nav_msgs.msg import Odometry

def yaw_from_quaternion(q):
    #yaw is turning arourd z axis
    # we need that cs the robot is moving around the floor so not 3D like odometry is given
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


class PointAndGo(Node):
    def __init__(self):
        super().__init__("point_and_go")

        self.goal_topic = "/goal_point"
        #odom  if for current robot position and orientation
        self.odom_topic = "/odom"
        #why did we change it from "/cmd_vel?"
        #we use "/cmd_vel_raw" -> controller
        #"/cmd_vel" -> safety filter
        self.cmd_topic = "/cmd_vel_raw"

        #control parameters
        self.k_lin = 0.8          #just velocity towards goal
        self.k_ang = 1.8          #velocity of turning towards goal
        self.max_lin = 0.6        #max lin vel
        self.max_ang = 1.5        #max ang vel
        self.goal_tolerance = 0.15  #when were closer then 15 cm to goal
        self.turn_in_place_angle = 0.35 #if robot is directed wrongly towards goal, turn it

        self.goal = None
        self.last_odom = None

        self.create_subscription(Point, self.goal_topic, self.on_goal, 10)
        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 20)
        self.cmd_pub = self.create_publisher(Twist, self.cmd_topic, 20)
        self.create_timer(0.05, self.control_loop)

        self.get_logger().info(f"Listening goal: {self.goal_topic}, odom: {self.odom_topic}, publishing: {self.cmd_topic}")

    def on_goal(self, msg: Point):
        self.goal = (msg.x, msg.y, msg.z)
        self.get_logger().info(f"New goal: x={msg.x:.2f}, y={msg.y:.2f}, z={msg.z:.2f}")

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
        z = od.pose.pose.position.z
        yaw = yaw_from_quaternion(od.pose.pose.orientation)

        gx, gy, gz = self.goal

        dx = gx - x
        dy = gy - y
        dist = math.hypot(dx, dy)

        if dist < self.goal_tolerance:
            self.stop_robot()
            self.get_logger().info("Goal reached -> stop.")
            self.goal = None
            return

        #this is for angular spped of the robot
        desired_yaw = math.atan2(dy, dx)
        yaw_err = desired_yaw - yaw
        # so that robot doesnt tur for ex for 350 rather -10
        yaw_err = (yaw_err + math.pi) % (2.0 * math.pi) - math.pi

        cmd = Twist()

        #if robot is very wrongly turned, first it needs to just turn in the place
        #if its just a little wrongy turned, it should stiil drive and simultaneously change the direction
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
