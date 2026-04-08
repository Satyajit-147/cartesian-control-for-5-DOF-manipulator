import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TwistStamped

class TwistToTwistStamped(Node):
    def __init__(self):
        super().__init__('twist_converter_node')
        
        # 1. Listen to the keyboard (Unstamped)
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.listener_callback,
            10)
            
        # 2. Speak to MoveIt Servo (Stamped)
        self.publisher_ = self.create_publisher(
            TwistStamped,
            '/servo_node/delta_twist_cmds',
            10)
            
        self.get_logger().info('Keyboard Bridge Started. Ready to drive!')

    def listener_callback(self, msg):
        # Create a new Stamped message
        stamped_msg = TwistStamped()
        
        # Add the Header (Critical for Servo)
        stamped_msg.header.stamp = self.get_clock().now().to_msg()
        stamped_msg.header.frame_id = 'base_link'  # Commands are relative to the Robot Base
        
        # Copy the velocity data
        stamped_msg.twist = msg
        
        # Publish
        self.publisher_.publish(stamped_msg)

def main(args=None):
    rclpy.init(args=args)
    node = TwistToTwistStamped()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()