import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class ConstantVelocityPublisher(Node):

    def __init__(self):
        # Initialize the node with a descriptive name
        super().__init__('constant_velocity_publisher')

        # Publisher for the Twist message on the same topic
        self.publisher_ = self.create_publisher(Twist, '/stylus_velocity', 10)

        # Timer: 100Hz = 0.01 seconds period
        self.timer_period = 0.01 
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

        self.get_logger().info('Constant Velocity Publisher Node started')

    def timer_callback(self):
        """
        This callback runs every 0.01s and publishes the fixed Twist message.
        """
        # 1. Create the fixed Twist message
        twist_msg = Twist()
        
        # Linear velocities
        twist_msg.linear.x = 0.0
        twist_msg.linear.y = 0.0
        twist_msg.linear.z = -1.0
        
        # Angular velocities
        twist_msg.angular.x = 0.0
        twist_msg.angular.y = 0.0
        twist_msg.angular.z = 0.0

        # 2. Publish the message
        self.publisher_.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ConstantVelocityPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        self.get_logger().info('Node stopping...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()