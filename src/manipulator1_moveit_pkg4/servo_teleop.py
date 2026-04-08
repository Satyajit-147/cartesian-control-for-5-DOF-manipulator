import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

class ServoTeleop(Node):
    def __init__(self):
        super().__init__('servo_teleop_node')
        # We publish to the topic the Servo node is listening to
        self.pub = self.create_publisher(TwistStamped, '/servo_node/delta_twist_cmds', 10)
        self.timer = self.create_timer(0.1, self.publish_twist) # Send command every 100ms

    def publish_twist(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link" # Commands are relative to the robot base
        
        # Move forward in X at 0.05 meters per second
        msg.twist.linear.x = 0.05 
        msg.twist.linear.y = 0.0
        msg.twist.linear.z = 0.0
        
        self.get_logger().info('Publishing Cartesian move command...')
        self.pub.publish(msg)

def main():
    rclpy.init()
    node = ServoTeleop()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()