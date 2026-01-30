#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2


class SafetyFilter(Node):
    def __init__(self):
        super().__init__("safety_filter")

        self.cmd_in = "/cmd_vel_raw"
        #final robot speeds
        self.cmd_out = "/cmd_vel"        
        self.cloud_topic = "/cloud_map"

        self.stop_distance = 0.6
        # just the front part of the laser
        self.front_fov_deg = 60.0       
        self.max_side = 0.6         
        self.z_min = -0.2             
        self.z_max = 1.0 
        
        self.last_cmd = Twist()
        self.has_cloud = False
        self.min_front = float("inf")

        self.create_subscription(Twist, self.cmd_in, self.on_cmd, 20)
        self.create_subscription(PointCloud2, self.cloud_topic, self.on_cloud, 20)
        self.pub = self.create_publisher(Twist, self.cmd_out, 20)

        self.create_timer(0.05, self.loop) 

    def on_cmd(self, msg: Twist):
        self.last_cmd = msg

    def on_cloud(self, msg: PointCloud2):
        half_fov = math.radians(self.front_fov_deg) * 0.5
        min_dist = float("inf")

        for x, y, z in point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
            #look forward
            if x <= 0.0:
                continue
            
            #filter by height 
            if z < self.z_min or z > self.z_max:
                continue

            #filter angle
            angle = math.atan2(y, x)
            if abs(angle) > half_fov:
                continue

            if abs(y) > self.max_side:
                continue

            d = math.hypot(x, y)
            if d < min_dist:
                min_dist = d

        self.min_front = min_dist
        self.has_cloud = True

    def loop(self):
        if not self.has_cloud:
            self.pub.publish(self.last_cmd)
            return

        cmd = Twist()
        cmd.linear.x = self.last_cmd.linear.x
        cmd.angular.z = self.last_cmd.angular.z
        
        #stop only when going forward
        if cmd.linear.x > 0.0 and self.min_front < self.stop_distance:
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        self.pub.publish(cmd)

def main():
    rclpy.init()
    node = SafetyFilter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
