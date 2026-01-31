#!/usr/bin/env python3

import rospy
import numpy as np
import tf 
from geometry_msgs.msg import PoseStamped, Vector3, Pose, Point, Quaternion, WrenchStamped, Vector3Stamped # Added Vector3Stamped
from omni_msgs.msg import OmniFeedback, OmniButtonEvent

class ForceFeedback:
    def __init__(self):
        rospy.init_node('force_feedback')

        # Initialize TF Listener
        self.tf_listener = tf.TransformListener()
        
        # Define the robot's base frame and end-effector (EE) frame
        # These should match your robot's URDF and MoveIt configuration
        self.robot_base_frame = rospy.get_param("~robot_base_frame", "world") 
        self.robot_ee_frame = rospy.get_param("~robot_ee_frame", "tool_frame") 

        rospy.loginfo(f"ForceFeedback: Listening for TF transform from '{self.robot_base_frame}' to '{self.robot_ee_frame}'")

        # Subscribers
        self.force_sensor_sub = rospy.Subscriber('/optoforce_0', WrenchStamped, self.force_sensor_callback)
        self.haptic_button_sub = rospy.Subscriber('/touch/button', OmniButtonEvent, self.button_callback)

        # Publishers
        self.force_feedback_pub = rospy.Publisher('/touch/force_feedback', OmniFeedback, queue_size=1)

        # Force Feedback State
        self.apply_force = False
        self.robot_ee_current_position = Vector3(0, 0, 0) 
        
        # We'll use a timer to periodically look up the robot's EE pose
        self.ee_pose_timer = rospy.Timer(rospy.Duration(0.01), self.ee_pose_lookup_callback) # Look up EE pose at 100 Hz

        # --- Calibration Parameters ---
        self.CALIBRATION_SAMPLES = 500 # Number of samples to collect for calibration
        self.is_calibrated = False
        self.calibration_readings = []
        self.bias_force = Vector3(0, 0, 0)
        self.calibration_count = 0

        rospy.loginfo(f"Starting force sensor calibration. Please ensure no contact with the sensor for {self.CALIBRATION_SAMPLES} samples.")


    def ee_pose_lookup_callback(self, event):
        """
        Timer callback to continuously look up the robot's end-effector pose
        and store it. This position is used for the 'position' field in OmniFeedback.
        """
        try:
            # For position, we typically want the *latest* transform, so rospy.Time(0) is appropriate.
            # No need for waitForTransform here, as we're not using this pose for force transformation timing.
            (translation, rotation) = self.tf_listener.lookupTransform(
                self.robot_base_frame,
                self.robot_ee_frame,
                rospy.Time(0) # Get the latest available transform
            )

            self.robot_ee_current_position.x = translation[0]
            self.robot_ee_current_position.y = translation[1]
            self.robot_ee_current_position.z = translation[2]

        except tf.LookupException as e:
            rospy.logdebug_throttle(1, f"TF Lookup Exception for EE pose: {e}. Is '{self.robot_ee_frame}' broadcasting correctly?")
        except tf.ConnectivityException as e:
            rospy.logwarn_throttle(1, f"TF Connectivity Exception for EE pose: {e}. Check if TF tree is valid between '{self.robot_base_frame}' and '{self.robot_ee_frame}'.")
        except tf.ExtrapolationException as e:
            rospy.logwarn_throttle(1, f"TF Extrapolation Exception for EE pose: {e}. Transform for '{self.robot_ee_frame}' is too old or in the future.")
        except Exception as e:
            rospy.logerr(f"An unexpected error occurred during EE TF lookup: {e}")

    def force_sensor_callback(self, msg):
        current_force = msg.wrench.force

        # --- Calibration Logic ---
        if not self.is_calibrated:
            self.calibration_readings.append(current_force)
            self.calibration_count += 1
            if self.calibration_count >= self.CALIBRATION_SAMPLES:
                # Calculate average bias
                x_sum = sum(f.x for f in self.calibration_readings)
                y_sum = sum(f.y for f in self.calibration_readings)
                z_sum = sum(f.z for f in self.calibration_readings)

                self.bias_force.x = x_sum / self.CALIBRATION_SAMPLES
                self.bias_force.y = y_sum / self.CALIBRATION_SAMPLES
                self.bias_force.z = z_sum / self.CALIBRATION_SAMPLES
                
                self.is_calibrated = True
                rospy.loginfo(f"Force sensor calibration complete. Bias: X={self.bias_force.x:.3f}, Y={self.bias_force.y:.3f}, Z={self.bias_force.z:.3f}")
                self.calibration_readings = [] # Clear readings to free memory
            else:
                rospy.loginfo_throttle(1, f"Calibrating force sensor: {self.calibration_count}/{self.CALIBRATION_SAMPLES} samples collected.")
                # During calibration, do not publish any force feedback
                self.publish_force_feedback(Vector3(0, 0, 0)) 
                return # Exit callback until calibration is done
        # --- End Calibration Logic ---

        # Apply bias compensation if calibrated
        biased_force_ee_frame = Vector3()
        biased_force_ee_frame.x = current_force.x - self.bias_force.x
        biased_force_ee_frame.y = current_force.y - self.bias_force.y
        biased_force_ee_frame.z = current_force.z - self.bias_force.z

        # --- Prepare Vector3Stamped for TF transformation ---
        force_vector_stamped = Vector3Stamped()
        force_vector_stamped.header.frame_id = self.robot_ee_frame # The frame of the original force sensor reading
        force_vector_stamped.header.stamp = msg.header.stamp       # The timestamp of when the force reading was taken
        force_vector_stamped.vector = biased_force_ee_frame        # The force vector data

        transformed_force_world_frame = Vector3(0,0,0)

        try:
            # Wait for the transform to become available at the exact timestamp of the force message.
            # This helps avoid ExtrapolationExceptions.
            self.tf_listener.waitForTransform(
                self.robot_base_frame,          # Target frame (e.g., "world" or "base_link")
                self.robot_ee_frame,            # Source frame (where the force is measured)
                msg.header.stamp,               # The timestamp for which we need the transform
                rospy.Duration(0.1)             # Timeout: Wait for up to 0.1 seconds
            )
            
            # Perform the transformation of the Vector3Stamped
            transformed_vector_stamped = self.tf_listener.transformVector3(
                self.robot_base_frame,       # Target frame for the vector
                force_vector_stamped         # The Vector3Stamped message containing the original vector and its frame/timestamp
            )
            
            # Extract the transformed Vector3 from the result
            transformed_force_world_frame = transformed_vector_stamped.vector 

            rospy.logdebug(f"Original force EE: ({biased_force_ee_frame.x:.3f}, {biased_force_ee_frame.y:.3f}, {biased_force_ee_frame.z:.3f})")
            rospy.logdebug(f"Transformed force World: ({transformed_force_world_frame.x:.3f}, {transformed_force_world_frame.y:.3f}, {transformed_force_world_frame.z:.3f})")

        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logwarn_throttle(1, f"Could not transform force vector: {e}")
            # If transformation fails, reset force to zero to prevent erratic feedback
            transformed_force_world_frame = Vector3(0, 0, 0)
        except Exception as e:
            rospy.logerr(f"An unexpected error occurred during force vector TF lookup: {e}")
            transformed_force_world_frame = Vector3(0, 0, 0) # Safety measure
            
        total_force_feedback = Vector3(0, 0, 0)

        # Apply your scaling to the transformed force, which is now in the world/base_link frame.
        # You will need to tune these signs and magnitudes for your specific haptic device
        # and how you want the force to feel.
        total_force_feedback.x = transformed_force_world_frame.x * 1 
        total_force_feedback.y = transformed_force_world_frame.y * 1  
        total_force_feedback.z = transformed_force_world_frame.z * 1
        
        rospy.loginfo(f"  total_force_feedback (world frame scaled): {total_force_feedback}")

        if self.apply_force == True:
            self.publish_force_feedback(total_force_feedback)
        else:
            total_force_feedback = Vector3(0, 0, 0)
            self.publish_force_feedback(total_force_feedback)
            
    def publish_force_feedback(self, force):
        feedback_msg = OmniFeedback()
        feedback_msg.force = force
        feedback_msg.position = self.robot_ee_current_position # Use the latest EE position for the haptic device's internal model
        self.force_feedback_pub.publish(feedback_msg)
    
    def button_callback(self, msg):
        if msg.grey_button == 1: 
            self.apply_force = True
            rospy.loginfo("Force feedback enabled (Grey Button Pressed)")
        elif msg.grey_button == 0: 
            self.apply_force = False
            rospy.loginfo("Force feedback disabled (Grey Button Released)")
        

if __name__ == '__main__':
    try:
        force_feedback = ForceFeedback()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass