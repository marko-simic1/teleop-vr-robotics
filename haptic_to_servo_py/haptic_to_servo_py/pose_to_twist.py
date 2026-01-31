import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from touch_msgs.msg import TouchButtonEvent 
import math

class StylusVelocityNode(Node):

    def __init__(self):
        super().__init__('stylus_velocity_node')
        self.publisher_ = self.create_publisher(Twist, '/stylus_velocity', 10)
        self.subscription = self.create_subscription(PoseStamped, '/touch/stylus_pose', self.pose_callback, 10)
        self.button_sub = self.create_subscription(TouchButtonEvent, '/touch/button', self.button_callback, 10)

        self.timer_period = 0.01 
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.is_active = False
        self.last_pose = None
        self.current_pose = None

        # --- Smoothing Parameters ---
        self.alpha_lin = 0.2   # Smoothing for Linear (Lower = smoother/slower)
        self.alpha_ang = 0.1   # Smoothing for Angular (Rotations are usually noisier)
        self.lin_deadband = 0.0015  # 1.5mm threshold
        
        self.prev_linear_vel = [0.0, 0.0, 0.0]
        self.prev_angular_vel = [0.0, 0.0, 0.0]

        self.get_logger().info('Stylus Velocity Node: Anti-Spike + Anti-Fall Edition started')
    
    def button_callback(self, msg: TouchButtonEvent):
        new_state = bool(msg.grey_button)
        if new_state and not self.is_active:
            self.last_pose = None
            self.prev_linear_vel = [0.0, 0.0, 0.0]
            self.prev_angular_vel = [0.0, 0.0, 0.0]
        self.is_active = new_state

    def pose_callback(self, msg: PoseStamped):
        self.current_pose = msg.pose

    def calculate_angular_velocity(self, q1, q2, dt):
        dot = q1.w*q2.w + q1.x*q2.x + q1.y*q2.y + q1.z*q2.z
        q1_w, q1_x, q1_y, q1_z = (q1.w, q1.x, q1.y, q1.z) if dot >= 0 else (-q1.w, -q1.x, -q1.y, -q1.z)
        
        q1_inv_w, q1_inv_x, q1_inv_y, q1_inv_z = q1_w, -q1_x, -q1_y, -q1_z
        dq_w = q2.w * q1_inv_w - q2.x * q1_inv_x - q2.y * q1_inv_y - q2.z * q1_inv_z
        dq_x = q2.w * q1_inv_x + q2.x * q1_inv_w + q2.y * q1_inv_z - q2.z * q1_inv_y
        dq_y = q2.w * q1_inv_y - q2.x * q1_inv_z + q2.y * q1_inv_w + q2.z * q1_inv_x
        dq_z = q2.w * q1_inv_z + q2.x * q1_inv_y - q2.y * q1_inv_x + q2.z * q1_inv_w

        dq_w = max(-1.0, min(1.0, dq_w))
        angle = 2.0 * math.acos(dq_w)
        sin_half_angle = math.sqrt(1.0 - dq_w * dq_w)
        if sin_half_angle < 1e-6: return 0.0, 0.0, 0.0
        
        factor = angle / (sin_half_angle * dt)
        return dq_x * factor, dq_y * factor, dq_z * factor

    def timer_callback(self):
        if not self.is_active or self.current_pose is None:
            return

        if self.last_pose is None:
            self.last_pose = self.current_pose
            return

        dt = self.timer_period
        
        # 1. Calculate raw displacement
        dx = self.current_pose.position.x - self.last_pose.position.x
        dy = self.current_pose.position.y - self.last_pose.position.y
        dz = self.current_pose.position.z - self.last_pose.position.z
        dist = math.sqrt(dx**2 + dy**2 + dz**2)

        # 2. Linear Velocity Noise Gate
        if dist > self.lin_deadband:
            raw_vx, raw_vy, raw_vz = dx/dt, dy/dt, dz/dt
        else:
            raw_vx, raw_vy, raw_vz = 0.0, 0.0, 0.0

        # 3. Apply Linear Low-Pass Filter (Prevents Spikes)
        self.prev_linear_vel[0] = self.alpha_lin * raw_vx + (1.0 - self.alpha_lin) * self.prev_linear_vel[0]
        self.prev_linear_vel[1] = self.alpha_lin * raw_vy + (1.0 - self.alpha_lin) * self.prev_linear_vel[1]
        self.prev_linear_vel[2] = self.alpha_lin * raw_vz + (1.0 - self.alpha_lin) * self.prev_linear_vel[2]

        # 4. Angular Velocity + Filter
        raw_wx, raw_wy, raw_wz = self.calculate_angular_velocity(self.last_pose.orientation, self.current_pose.orientation, dt)
        
        self.prev_angular_vel[0] = self.alpha_ang * raw_wx + (1.0 - self.alpha_ang) * self.prev_angular_vel[0]
        self.prev_angular_vel[1] = self.alpha_ang * raw_wy + (1.0 - self.alpha_ang) * self.prev_angular_vel[1]
        self.prev_angular_vel[2] = self.alpha_ang * raw_wz + (1.0 - self.alpha_ang) * self.prev_angular_vel[2]

        self.prev_angular_vel[0] = 0.0
        self.prev_angular_vel[1] = 0.0
        self.prev_angular_vel[2] = 0.0

        # 5. Build and Publish Message
        twist_msg = Twist()
        twist_msg.linear.x, twist_msg.linear.y, twist_msg.linear.z = self.prev_linear_vel
        twist_msg.angular.x, twist_msg.angular.y, twist_msg.angular.z = self.prev_angular_vel

        self.last_pose = self.current_pose
        self.publisher_.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = StylusVelocityNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()