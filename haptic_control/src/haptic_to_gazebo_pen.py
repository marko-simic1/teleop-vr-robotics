#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Vector3, Pose, Point, Quaternion
from gazebo_msgs.srv import SetModelState, SetModelStateRequest
from gazebo_msgs.msg import ModelState, ContactsState, ContactState
from omni_msgs.msg import OmniFeedback
import tf.transformations as transformations # Import for quaternion and matrix operations

class HapticToGazeboPen:
    def __init__(self):
        rospy.init_node('haptic_to_gazebo_pen')

        # Subscribers
        # Subscribes to the haptic device's pose
        self.haptic_pose_sub = rospy.Subscriber('/touch/stylus_pose', PoseStamped, self.haptic_pose_callback)
        # Subscribes to Gazebo contact states for the haptic pen
        self.contact_sub = rospy.Subscriber('/pen_contact', ContactsState, self.contact_callback)
        
        # Publishers
        # Publishes force feedback to the haptic device driver
        self.force_feedback_pub = rospy.Publisher('/touch/force_feedback', OmniFeedback, queue_size=1)

        self.contact_pub = rospy.Publisher('/pen_contact_viz', ContactState, queue_size=1)
        
        # Services
        # Service client to set the model state of the pen in Gazebo
        self.set_model_state_proxy = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        
        # Parameters
        self.pen_name = "haptic_pen"  # Name of the pen model in Gazebo
        self.desired_tip_pose = None       # Stores the latest tip pose received from the haptic device

        self.counter_pose = 0
        self.counter_contact = 0

        # Define force feedback properties for different objects
        self.object_properties = {
            "interactive_sphere_model": { # Make sure this matches your URDF's robot name
                "stiffness": 500.0,  # N/m
            },
            "another_cube_model": { # Example for another object
                "stiffness": 800.0,
            }
        }
        # Default properties if an object isn't found in the dictionary
        self.default_properties = {"stiffness": 0.0}

    def haptic_pose_callback(self, msg):
        """
        Callback for the haptic device's pose.
        Updates the desired tip pose for the Gazebo pen model.
        """
        self.counter_pose = self.counter_pose + 1
        if self.counter_pose >= 10: # Increased frequency for smoother updates
            self.desired_tip_pose = msg.pose
            self.publish_gazebo_pose()
            self.counter_pose = 0

    def contact_callback(self, msg):
        """
        Callback for Gazebo contact states.
        Calculates and publishes force feedback based on detected contacts.
        (Contact feedback logic remains the same as it correctly identifies collisions)
        """
        rospy.loginfo("Received ContactsState message.")
        if msg.states:
            for contact_state in msg.states:
                if (contact_state.collision1_name != "ground_plane::link::collision") and \
                   (contact_state.collision2_name != "ground_plane::link::collision"): 
                    rospy.loginfo(f"  Collision 1 Name: {contact_state.collision1_name}")
                    rospy.loginfo(f"  Collision 2 Name: {contact_state.collision2_name}")
        else:
            rospy.loginfo("  No contact states reported in this message.")
         
        self.counter_contact = self.counter_contact + 1
        if self.counter_contact >= 10:
            
            #rospy.loginfo("force callback")
            # Initialize total force and position for feedback
            total_force_feedback = Vector3(0, 0, 0)
            contact_position_feedback = Vector3(0, 0, 0)
            num_contacts = 0

            if not msg.states:  # No contacts detected
                self.publish_force_feedback(Vector3(0, 0, 0), Vector3(0, 0, 0)) # Send zero force
                return
            
            for contact_state in msg.states:
                colliding_object_name = None
                
                # Identify the other object involved in the collision
                # "haptic_pen::pen_tip_link::pen_tip_link_fixed_joint_lump__pen_tip_collision_collision"
                # "haptic_pen::base_link::base_link_fixed_joint_lump__pen_tip_collision_collision"
                if (contact_state.collision1_name == "haptic_pen::pen_tip_link::pen_tip_link_fixed_joint_lump__pen_tip_collision_collision") or \
                   (contact_state.collision2_name == "haptic_pen::pen_tip_link::pen_tip_link_fixed_joint_lump__pen_tip_collision_collision"):
                    rospy.loginfo("Contacts with pen detected")
                    #rospy.loginfo(f"  total_wrench: {contact_state.total_wrench}")
                    #rospy.loginfo(f"  contact_positions: {contact_state.contact_positions}")
                    #rospy.loginfo(f"  depths: {contact_state.depths}")
                    self.contact_pub.publish(contact_state)

                    current_stiffness = self.object_properties["interactive_sphere_model"]["stiffness"]

                    # A contact can have multiple contact points. Sum their contributions.
                    for i, contact_point in enumerate(contact_state.contact_positions):

                        # Accumulate forces and positions
                        total_force_feedback.x += contact_state.wrenches[0].force.x * 100
                        total_force_feedback.y += contact_state.wrenches[0].force.y * 100
                        total_force_feedback.z += contact_state.wrenches[0].force.z * 100

                        contact_position_feedback.x += contact_point.x
                        contact_position_feedback.y += contact_point.y
                        contact_position_feedback.z += contact_point.z
                        num_contacts += 1
            
            if num_contacts > 0:
                # Average the contact position if there are multiple contact points
                contact_position_feedback.x /= num_contacts
                contact_position_feedback.y /= num_contacts
                contact_position_feedback.z /= num_contacts
            
            # Publish the total calculated force and average contact position
            self.publish_force_feedback(total_force_feedback, contact_position_feedback)
            # rospy.loginfo(f"  total_force_feedback: {total_force_feedback}")
            # rospy.loginfo(f"  contact_position_feedback: {contact_position_feedback}")

            self.counter_contact = 0

    def publish_force_feedback(self, force, position):
        """
        Publishes the calculated force and position feedback to the haptic device.
        """
        feedback_msg = OmniFeedback()
        feedback_msg.force = force
        feedback_msg.position = position
        self.force_feedback_pub.publish(feedback_msg)

    def publish_gazebo_pose(self):
        """
        Calculates the required base_link pose to make the tip follow the desired_tip_pose
        and sends it to Gazebo to update the pen model's position.
        """
        if self.desired_tip_pose is not None:
            # 1. Get the desired tip position and orientation from the haptic device
            desired_tip_pos_global = np.array([self.desired_tip_pose.position.x,
                                               self.desired_tip_pose.position.y,
                                               self.desired_tip_pose.position.z])
            
            haptic_stylus_quat = np.array([self.desired_tip_pose.orientation.x,
                                           self.desired_tip_pose.orientation.y,
                                           self.desired_tip_pose.orientation.z,
                                           self.desired_tip_pose.orientation.w])

            # 5. Construct the ModelState message for the haptic_pen model (which sets its base_link's pose)
            model_state = ModelState()
            model_state.model_name = self.pen_name
            
            # # Set the position of the base_link
            # model_state.pose.position.x = base_pos_global[0]
            # model_state.pose.position.y = base_pos_global[1]
            # model_state.pose.position.z = base_pos_global[2]

            # # Set the orientation of the base_link
            # model_state.pose.orientation.x = base_link_quat_gazebo[0]
            # model_state.pose.orientation.y = base_link_quat_gazebo[1]
            # model_state.pose.orientation.z = base_link_quat_gazebo[2]
            # model_state.pose.orientation.w = base_link_quat_gazebo[3]

            model_state.pose.position.x = desired_tip_pos_global[0]
            model_state.pose.position.y = desired_tip_pos_global[1]
            model_state.pose.position.z = desired_tip_pos_global[2]

            # Set the orientation of the base_link
            model_state.pose.orientation.x = haptic_stylus_quat[0]
            model_state.pose.orientation.y = haptic_stylus_quat[1]
            model_state.pose.orientation.z = haptic_stylus_quat[2]
            model_state.pose.orientation.w = haptic_stylus_quat[3]
            
            try:
                req = SetModelStateRequest()
                req.model_state = model_state
                resp = self.set_model_state_proxy(req)
                if not resp.success:
                    rospy.logwarn(f"Failed to set model state for {self.pen_name}: {resp.status_message}")
            except rospy.ServiceException as e:
                rospy.logerr(f"Service call to set_model_state failed: {e}")

if __name__ == '__main__':
    try:
        haptic_to_gazebo_pen = HapticToGazeboPen()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass