#!/usr/bin/env python3

import math


import rclpy

from rclpy.node import Node

from geometry_msgs.msg import Twist

from sensor_msgs.msg import LaserScan



class SafetyFilter(Node):

    def _init_(self):

        super()._init_("safety_filter")


        self.cmd_in = "/cmd_vel_raw"

        #final robot speeds

        self.cmd_out = "/cmd_vel"        

        self.scan_topic = "/scan"


        self.stop_distance = 0.6

        # just the front part of the laser

        self.front_fov_deg = 60.0       


        self.last_cmd = Twist()

        self.has_scan = False

        self.min_front = float("inf")


        self.create_subscription(Twist, self.cmd_in, self.on_cmd, 20)

        self.create_subscription(LaserScan, self.scan_topic, self.on_scan, 20)

        self.pub = self.create_publisher(Twist, self.cmd_out, 20)


        self.create_timer(0.05, self.loop) 


        self.get_logger().info(

            f"SafetyFilter: in={self.cmd_in}, out={self.cmd_out}, scan={self.scan_topic}, "

            f"stop_distance={self.stop_distance}m, front_fov={self.front_fov_deg}deg"

        )


    def on_cmd(self, msg: Twist):

        self.last_cmd = msg


    def on_scan(self, msg: LaserScan):

        fov = math.radians(self.front_fov_deg)

        a_min = msg.angle_min

        inc = msg.angle_increment


        if inc <= 0.0 or len(msg.ranges) == 0:

            return


        i0 = int(((-fov / 2) - a_min) / inc)

        i1 = int((( fov / 2) - a_min) / inc)


        i0 = max(0, min(len(msg.ranges) - 1, i0))

        i1 = max(0, min(len(msg.ranges) - 1, i1))

        if i1 < i0:

            i0, i1 = i1, i0


        m = float("inf")

        for r in msg.ranges[i0:i1+1]:

            if r is None:

                continue

            if math.isfinite(r) and r > msg.range_min and r < msg.range_max:

                if r < m:

                    m = r


        self.min_front = m

        self.has_scan = True


    def loop(self):

        if not self.has_scan:

            self.pub.publish(self.last_cmd)

            return


        cmd = Twist()

        cmd.linear.x = self.last_cmd.linear.x

        cmd.angular.z = self.last_cmd.angular.z


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



if _name_ == "_main_":

    main()
