#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Vector3, Pose, Point, Quaternion, WrenchStamped 
from omni_msgs.msg import OmniFeedback, OmniButtonEvent
from tf.transformations import quaternion_inverse, quaternion_multiply, euler_from_quaternion, quaternion_from_euler
import tf # Import the tf library
import tf2_ros # For TF listener
import geometry_msgs.msg # For TransformStamped
import tf2_geometry_msgs # For do_transform_pose

class Controller:
    def __init__(self):
        rospy.init_node('Controller')

        # 1. Define all static parameters and frame IDs first
        self.world_frame_id = "world" # Or your actual world frame
        self.robot_tool_frame_id = "tool_frame" # The robot's end-effector frame

        # 2. Initialize TF Broadcaster and Listener
        self.tf_broadcaster = tf.TransformBroadcaster()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        
        # 3. Initialize all instance variables with default states
        self.last_haptic_pose_world = None 
        self.enable_control = False
        self.accumulated_pose = Pose()
        self.accumulated_pose.orientation.w = 1.0 # Initialize to identity quaternion (default, will be overwritten)
        
        # New flag: True after the very first synchronization with the robot's initial pose
        self.is_initial_robot_sync_done = False 

        # 4. Set up rate control
        self.tf_publish_rate_hz = 50 
        self.tf_publish_interval = rospy.Duration(1.0 / self.tf_publish_rate_hz)
        self.last_tf_publish_time = rospy.Time.now()

        # 5. Initialize Publishers (they don't trigger callbacks on creation)
        self.relative_transform_pub = rospy.Publisher('/robot_arm/relative_pose_command', PoseStamped, queue_size=10)

        rospy.loginfo("Controller node ready. Initializing...")

        # 6. Finally, initialize Subscribers. Messages will start arriving NOW.
        self.haptic_pose_sub = rospy.Subscriber('/touch/stylus_pose', PoseStamped, self.pose_callback)
        self.haptic_button_sub = rospy.Subscriber('/touch/button', OmniButtonEvent, self.button_callback) 
        
        rospy.loginfo("Subscribers initialized. Waiting for control enable and robot tool frame TF for initial synchronization...")


    def transform_pose_to_world(self, pose_stamped_msg):
        try:
            # Wait for the transform to be available
            self.tf_buffer.can_transform(self.world_frame_id, pose_stamped_msg.header.frame_id, 
                                         pose_stamped_msg.header.stamp, rospy.Duration(0.1)) 
            
            transform = self.tf_buffer.lookup_transform(self.world_frame_id, pose_stamped_msg.header.frame_id, 
                                                        pose_stamped_msg.header.stamp)
            
            transformed_pose = tf2_geometry_msgs.do_transform_pose(pose_stamped_msg, transform)
            return transformed_pose.pose
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            # rospy.logdebug_throttle(1, f"Failed to transform pose from {pose_stamped_msg.header.frame_id} to {self.world_frame_id}: {e}")
            return None

    def get_robot_current_pose_in_world(self):
        try:
            transform_stamped = self.tf_buffer.lookup_transform(self.world_frame_id, self.robot_tool_frame_id, 
                                                                rospy.Time(0), rospy.Duration(0.5)) 
            
            robot_pose = Pose()
            robot_pose.position.x = transform_stamped.transform.translation.x
            robot_pose.position.y = transform_stamped.transform.translation.y
            robot_pose.position.z = transform_stamped.transform.translation.z
            robot_pose.orientation.x = transform_stamped.transform.rotation.x
            robot_pose.orientation.y = transform_stamped.transform.rotation.y
            robot_pose.orientation.z = transform_stamped.transform.rotation.z
            robot_pose.orientation.w = transform_stamped.transform.rotation.w
            
            return robot_pose
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException) as e:
            # rospy.logdebug_throttle(1, f"Failed to lookup transform from {self.world_frame_id} to {self.robot_tool_frame_id}: {e}")
            return None


    def pose_callback(self, msg):
        current_haptic_pose_world = self.transform_pose_to_world(msg)
        if current_haptic_pose_world is None:
            return 

        if not self.enable_control:
            # When control is disabled, we just ignore pose updates for accumulation
            return 

        # --- Control is enabled from this point ---
        
        # Scenario 1: Initial synchronization when control is enabled for the very first time (node startup)
        if not self.is_initial_robot_sync_done:
            robot_current_pose_in_world = self.get_robot_current_pose_in_world()
            if robot_current_pose_in_world is None:
                rospy.logwarn_throttle(2, f"Control enabled but cannot get robot's initial EE pose from TF. Waiting for transform from '{self.world_frame_id}' to '{self.robot_tool_frame_id}'.")
                return 

            # Initialize accumulated pose to robot's current pose ONLY ONCE
            self.accumulated_pose = robot_current_pose_in_world 
            self.is_initial_robot_sync_done = True
            rospy.loginfo("Initial synchronization complete: Accumulated pose set to robot's current EE pose.")
            # FALL THROUGH to set last_haptic_pose_world for this first message after initial sync

        # Scenario 2: Establish new haptic reference after control was just (re)enabled.
        # This includes the very first message after initial robot sync, and the first message
        # after control was disabled and then re-enabled.
        if self.last_haptic_pose_world is None: 
            self.last_haptic_pose_world = current_haptic_pose_world
            rospy.loginfo("Haptic reference point established. Accumulated pose will continue from its current state.")
            return # Skip delta application for this first message after (re)enablement

        # Scenario 3: Normal operation - calculate and apply deltas
        pose1_world = self.last_haptic_pose_world
        pose2_world = current_haptic_pose_world
        
        translation_delta_world = self.get_translation_diff(pose1_world, pose2_world)
        rotation_delta_world_quat = self.get_rotation_diff(pose1_world, pose2_world)
        
        # Apply translation difference to accumulated pose
        self.accumulated_pose.position.x += translation_delta_world[0]
        self.accumulated_pose.position.y += translation_delta_world[1]
        self.accumulated_pose.position.z += translation_delta_world[2]

        # Apply rotation difference to accumulated pose
        current_orientation_acc = [self.accumulated_pose.orientation.x,
                                   self.accumulated_pose.orientation.y,
                                   self.accumulated_pose.orientation.z,
                                   self.accumulated_pose.orientation.w]
        
        new_orientation_acc = quaternion_multiply(rotation_delta_world_quat, current_orientation_acc)
        
        self.accumulated_pose.orientation.x = new_orientation_acc[0]
        self.accumulated_pose.orientation.y = new_orientation_acc[1]
        self.accumulated_pose.orientation.z = new_orientation_acc[2]
        self.accumulated_pose.orientation.w = new_orientation_acc[3]

        # Rate limit TF broadcasting and relative pose publishing
        if (rospy.Time.now() - self.last_tf_publish_time) > self.tf_publish_interval:
            self.publish_accumulated_pose(msg.header) 
            self.broadcast_accumulated_transform(msg.header)
            self.last_tf_publish_time = rospy.Time.now()

        self.last_haptic_pose_world = current_haptic_pose_world

    def button_callback(self, msg):
        # Only change state if button press changes it
        if msg.white_button == 1 and not self.enable_control:
            self.enable_control = True
            # Crucial: Reset last_haptic_pose_world to None.
            # This signals pose_callback that the next incoming haptic pose is the new reference.
            self.last_haptic_pose_world = None 
            rospy.loginfo("Control Enabled! Waiting for first haptic input to establish new relative start.")
        elif msg.white_button == 0 and self.enable_control:
            self.enable_control = False
            # When disabled, the accumulated_pose is *not* reset. It holds its value.
            # Also reset last_haptic_pose_world to None to prepare for next re-enablement.
            self.last_haptic_pose_world = None
            rospy.loginfo("Control Disabled! Accumulated pose holding last position.")

    def publish_accumulated_pose(self, header):
        accumulated_pose_stamped = PoseStamped()
        accumulated_pose_stamped.header.frame_id = self.world_frame_id 
        accumulated_pose_stamped.header.stamp = header.stamp
        accumulated_pose_stamped.pose = self.accumulated_pose
        self.relative_transform_pub.publish(accumulated_pose_stamped)
        
    def broadcast_accumulated_transform(self, header):
        translation = (self.accumulated_pose.position.x,
                       self.accumulated_pose.position.y,
                       self.accumulated_pose.position.z)
        
        rotation = (self.accumulated_pose.orientation.x,
                    self.accumulated_pose.orientation.y,
                    self.accumulated_pose.orientation.z,
                    self.accumulated_pose.orientation.w)
        
        self.tf_broadcaster.sendTransform(translation,
                                          rotation,
                                          header.stamp, 
                                          "accumulated_object_frame", 
                                          self.world_frame_id) 
        
    def get_translation_diff(self, pose1, pose2):
        p1 = np.array([pose1.position.x, pose1.position.y, pose1.position.z])
        p2 = np.array([pose2.position.x, pose2.position.y, pose2.position.z])

        return p2 - p1
    
    def get_rotation_diff(self, pose1, pose2): 
        q1 = [pose1.orientation.x, pose1.orientation.y, pose1.orientation.z, pose1.orientation.w]
        q2 = [pose2.orientation.x, pose2.orientation.y, pose2.orientation.z, pose2.orientation.w]

        q1_inv = quaternion_inverse(q1)
        q_rel = quaternion_multiply(q2, q1_inv)  # rotation from pose1 to pose2

        return q_rel # quaternion
    
    def get_rotation_diff_euler(self, pose1, pose2):
        q_rel = self.get_rotation_diff(pose1, pose2)
        return euler_from_quaternion(q_rel)  # returns (roll, pitch, yaw)

if __name__ == '__main__':
    try:
        controller = Controller()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass