#!/usr/bin/env python3

import sys
import time
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi
from std_srvs.srv import Empty
import numpy as np
import tf # Import the tf library for TransformListener
import tf2_ros # Import tf2_ros for TransformBroadcaster
from tf.transformations import quaternion_multiply, quaternion_from_euler, euler_from_quaternion, quaternion_inverse

class KinovaMoveItTeleop(object):
    """
    KinovaMoveItTeleop: Controls the Kinova robotic arm by listening to a TF transform
    broadcasted by a haptic controller (e.g., your Controller script).
    It expects a transform named 'accumulated_object_frame' relative to a 'world' or 'robot_base' frame
    and commands the robot's end-effector to reach that absolute pose.
    """
    def __init__(self):
        # Initialize MoveIt Commander and ROS node
        super(KinovaMoveItTeleop, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        rospy.init_node('kinova_moveit_teleop', anonymous=True)

        namespace = "/my_gen3/" # This namespace is used for other parameters as well

        # Flag to indicate if the MoveIt interface is successfully initialized
        self.is_init_success = False # Initialize to False, set to True only on full success

        # Retrieve parameters from the ROS parameter server
        try:
            self.is_gripper_present = rospy.get_param(namespace + "is_gripper_present", False)
            if self.is_gripper_present:
                gripper_joint_names = rospy.get_param(namespace + "gripper_joint_names", [])
                self.gripper_joint_name = gripper_joint_names[0]
            else:
                self.gripper_joint_name = ""
            self.degrees_of_freedom = rospy.get_param(namespace + "degrees_of_freedom", 7)

            # Define the MoveIt group names (e.g., "arm" for the robotic arm)
            arm_group_name = "arm"
            
            # Initialize RobotCommander, Scene, and MoveGroupCommander
            # Pass the full parameter name including the namespace for robot_description
            self.robot = moveit_commander.RobotCommander(robot_description=namespace + "robot_description") 
            self.scene = moveit_commander.PlanningSceneInterface(ns=namespace)
            self.arm_group = moveit_commander.MoveGroupCommander(arm_group_name, ns=namespace)

            # Publisher for displaying planned trajectories (useful for debugging)
            self.display_trajectory_publisher = rospy.Publisher(
                namespace + 'move_group/display_planned_path',
                moveit_msgs.msg.DisplayTrajectory,
                queue_size=20
            )

            # If a gripper is present, initialize its MoveGroupCommander
            if self.is_gripper_present:
                gripper_group_name = "gripper"
                self.gripper_group = moveit_commander.MoveGroupCommander(gripper_group_name, ns=namespace)

            rospy.loginfo("Initializing Kinova MoveIt Teleop node in namespace " + namespace)

            # --- CRITICAL: Define robot_base_frame BEFORE its usage ---
            # The base frame of your robot. This is the frame your robot's poses are typically given relative to.
            # Common options: "base_link", "world", "kinova_link_base".
            self.robot_base_frame = self.arm_group.get_planning_frame()
            rospy.loginfo(f"Robot planning frame: {self.robot_base_frame}")

            # Get the robot's end-effector link name from MoveIt
            self.ee_link = self.arm_group.get_end_effector_link()
            if not self.ee_link:
                rospy.logerr("Could not get end-effector link name from MoveIt. Is it configured correctly?")
                self.is_init_success = False 
                return # Exit init if critical component is missing

            rospy.loginfo(f"Robot End-Effector Link: {self.ee_link}")

            # --- Wait for robot's initial pose to become available ---
            initial_robot_pose = None
            max_attempts = 20 
            for i in range(max_attempts):
                current_pose_stamped = self.arm_group.get_current_pose()
                initial_robot_pose = current_pose_stamped.pose
                if initial_robot_pose.position.x != 0.0 or \
                   initial_robot_pose.position.y != 0.0 or \
                   initial_robot_pose.position.z != 0.0:
                    rospy.loginfo(f"Robot initial pose obtained after {i+1} attempts: {initial_robot_pose}")
                    break
                rospy.logwarn(f"Waiting for valid robot initial pose (attempt {i+1}/{max_attempts}). Current: {initial_robot_pose}")
                rospy.sleep(0.5) 

            if initial_robot_pose is None or (initial_robot_pose.position.x == 0.0 and \
                                              initial_robot_pose.position.y == 0.0 and \
                                              initial_robot_pose.position.z == 0.0):
                rospy.logerr("Could not get a valid initial robot end-effector pose after multiple attempts. Teleoperation might not start correctly.")
                self.is_init_success = False
                return # Exit init if critical component is missing

            self.initial_robot_ee_pose = initial_robot_pose
            
            # --- Publisher for the robot's initial EE pose ---
            self.initial_robot_pose_pub = rospy.Publisher(
                "/robot_arm/initial_ee_pose",
                geometry_msgs.msg.PoseStamped,
                queue_size=1,
                latch=True 
            )
            initial_pose_msg = geometry_msgs.msg.PoseStamped()
            initial_pose_msg.header.stamp = rospy.Time.now()
            initial_pose_msg.header.frame_id = self.robot_base_frame 
            initial_pose_msg.pose = self.initial_robot_ee_pose
            self.initial_robot_pose_pub.publish(initial_pose_msg)
            rospy.loginfo("Published initial robot end-effector pose.")

            # Initialize TF Listener
            self.tf_listener = tf.TransformListener()
            
            # Initialize TF Broadcaster for debugging the target pose
            self.tf_broadcaster = tf2_ros.TransformBroadcaster() 
            self.debug_target_frame_id = "moveit_command_target_pose" 

            # The frame ID of the accumulated pose broadcasted by the haptic controller.
            self.haptic_target_frame = "accumulated_object_frame"

            # --- Adjustments for Overshooting and Trajectory Errors ---
            self.min_move_command_interval = rospy.Duration(1.0) 
            self.max_linear_speed = 1.0 
            self.max_angular_speed = 1.0 
            self.arm_group.set_goal_position_tolerance(0.01)  # 1 cm tolerance
            self.arm_group.set_goal_orientation_tolerance(0.02) # approx 1.1 degrees
            self.arm_group.set_planning_time(0.02) # Try 0.2-0.5 seconds for planning
            self.arm_group.set_num_planning_attempts(5) # Allow more attempts if planning fails
            self.arm_group.set_max_velocity_scaling_factor(1.0) # Start low (e.g., 20% of max speed)
            self.arm_group.set_max_acceleration_scaling_factor(1.0) # Even lower for acceleration for smoother stops

            self.tf_timer = rospy.Timer(rospy.Duration(0.02), self.tf_lookup_callback) 

            self.is_init_success = True 
            rospy.loginfo("Kinova MoveIt Teleop node ready. Waiting for TF commands...")

        except Exception as e:
            rospy.logerr(f"Failed to initialize KinovaMoveItTeleop: {e}")
            self.is_init_success = False 

        self.last_move_command_time = rospy.Time.now() 

    def tf_lookup_callback(self, event):
        """
        Timer callback function to periodically look up the desired TF transform.
        """
        if not self.is_init_success:
            rospy.logwarn("MoveIt interface not initialized. Skipping TF lookup.")
            return

        # Rate limit the actual motion commands sent to MoveIt
        if (rospy.Time.now() - self.last_move_command_time) < self.min_move_command_interval:
            return

        try:
            self.tf_listener.waitForTransform(
                self.robot_base_frame,
                self.haptic_target_frame,
                rospy.Time(0), 
                rospy.Duration(0.5) 
            )
            
            (translation, rotation) = self.tf_listener.lookupTransform(
                self.robot_base_frame,
                self.haptic_target_frame,
                rospy.Time(0) 
            )

            target_pose = geometry_msgs.msg.Pose()
            target_pose.position.x = translation[0]
            target_pose.position.y = translation[1]
            target_pose.position.z = translation[2]

            target_pose.orientation.x = rotation[0]
            target_pose.orientation.y = rotation[1]
            target_pose.orientation.z = rotation[2]
            target_pose.orientation.w = rotation[3]

            # Broadcast the target pose as a TF transform for RViz debugging
            t = geometry_msgs.msg.TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = self.robot_base_frame
            t.child_frame_id = self.debug_target_frame_id 

            t.transform.translation.x = target_pose.position.x
            t.transform.translation.y = target_pose.position.y
            t.transform.translation.z = target_pose.position.z

            t.transform.rotation.x = target_pose.orientation.x
            t.transform.rotation.y = target_pose.orientation.y
            t.transform.rotation.z = target_pose.orientation.z
            t.transform.rotation.w = target_pose.orientation.w

            self.tf_broadcaster.sendTransform(t)

            # Command MoveIt to move to the new target pose
            success = self.go_to_pose(target_pose)
            if success:
                self.last_move_command_time = rospy.Time.now()
            else:
                rospy.logwarn_throttle(1, "Failed to plan or execute movement to target pose. MoveIt might be busy or goal unreachable.")

        except tf.LookupException as e:
            rospy.logdebug(f"TF Lookup Exception: {e}. Waiting for transform '{self.haptic_target_frame}' to '{self.robot_base_frame}'.")
        except tf.ConnectivityException as e:
            rospy.logerr(f"TF Connectivity Exception: {e}. Check if TF tree is valid.")
        except tf.ExtrapolationException as e:
            rospy.logwarn_throttle(1, f"TF Extrapolation Exception: {e}. Transform is too old or in the future. "
                          "Ensure timestamps are synchronized and the haptic node is broadcasting at a sufficient rate.")
        except Exception as e:
            rospy.logerr(f"An unexpected error occurred during TF lookup: {e}")


    def go_to_pose(self, pose):
        """
        Plans and executes a movement to a specified Cartesian pose.
        """
        self.arm_group.set_pose_target(pose)
        
        # Allow replanning if initial plan fails
        self.arm_group.allow_replanning(True) 
        
        # --- MODIFIED FOR DEBUGGING: Plan the trajectory explicitly ---
        rospy.loginfo(f"Attempting to plan to pose: {pose.position.x:.3f}, {pose.position.y:.3f}, {pose.position.z:.3f}")
        
        # Call plan()
        # The result is a tuple: (success, plan, planning_time, error_code)
        plan_success, plan, planning_time, error_code = self.arm_group.plan()

        if plan_success:
            rospy.loginfo("Planning successful! Publishing trajectory to RViz and executing...")
            
            # Create a DisplayTrajectory message to visualize the plan in RViz
            display_trajectory = moveit_msgs.msg.DisplayTrajectory()
            display_trajectory.trajectory_start = self.robot.get_current_state()
            display_trajectory.trajectory.append(plan)
            self.display_trajectory_publisher.publish(display_trajectory)
            
            # Execute the planned trajectory
            # Use arm_group.go(wait=False) equivalent for non-blocking execution
            success_execution = self.arm_group.execute(plan, wait=False) 
            return success_execution
        else:
            rospy.logwarn(f"Planning failed! Error code: {error_code.val}. Will not execute.")
            # Log more details if needed
            if error_code.val == moveit_msgs.msg.MoveItErrorCodes.NO_IK_SOLUTION:
                rospy.logwarn("Reason: No Inverse Kinematics solution found for the target pose.")
            elif error_code.val == moveit_msgs.msg.MoveItErrorCodes.TIMED_OUT:
                rospy.logwarn("Reason: Planning timed out.")
            elif error_code.val == moveit_msgs.msg.MoveItErrorCodes.PLANNING_FAILED:
                rospy.logwarn("Reason: General planning failure.")
            elif error_code.val == moveit_msgs.msg.MoveItErrorCodes.INVALID_ROBOT_STATE:
                rospy.logwarn("Reason: Current robot state is invalid for planning.")
            elif error_code.val == moveit_msgs.msg.MoveItErrorCodes.COLLISION_DETECTED:
                rospy.logwarn("Reason: Target pose or path is in collision.")


            return False # Indicate planning failure

def main():
    # Create an instance of the KinovaMoveItTeleop controller
    teleop_controller = KinovaMoveItTeleop()

    if teleop_controller.is_init_success:
        # Keep the node running, waiting for incoming TF commands via the timer
        rospy.spin()
    else:
        rospy.logerr("Kinova MoveIt Teleop node failed to initialize. Exiting.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass