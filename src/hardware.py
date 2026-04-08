import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64

class HardwareSplitter(Node):
    def __init__(self):
        super().__init__('hardware_splitter_node')

        # --- CONFIGURATION ---
        # 1. Define the input topic
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10)

        # 2. Define your hardware topics (Change these names to match your Arduino/Driver)
        self.pub_joint_1 = self.create_publisher(Float64, '/hardware/motor_base', 10)
        self.pub_joint_2 = self.create_publisher(Float64, '/hardware/motor_shoulder', 10)
        self.pub_joint_3 = self.create_publisher(Float64, '/hardware/motor_elbow', 10)

        # 3. Define the mapping: Joint Name -> Publisher
        # This ensures the right data goes to the right motor, even if the list order changes.
        self.joint_map = {
            'base_link_j_base_plate':       self.pub_joint_1,
            'side_plate1_j_side_plate12':   self.pub_joint_2,
            'motor_holder_j_side_plate_12': self.pub_joint_3
        }

        self.get_logger().info("Hardware Splitter Node Started. Waiting for joint_states...")

    def joint_state_callback(self, msg):
        # Iterate through the names in the incoming message
        # msg.name is a list like ['joint_a', 'joint_b']
        # msg.position is a list like [0.5, 1.2]
        
        try:
            for i, name in enumerate(msg.name):
                # Check if this joint is one we care about
                if name in self.joint_map:
                    # Create the standard message for hardware
                    cmd_msg = Float64()
                    
                    # Extract the position (Angle in Radians)
                    angle_rad = msg.position[i]
                    
                    # OPTIONAL: Convert Radians to Degrees if your hardware needs it
                    # angle_deg = angle_rad * (180.0 / 3.14159)
                    # cmd_msg.data = angle_deg
                    
                    cmd_msg.data = angle_rad # Sending Radians by default

                    # Publish to the specific topic for this joint
                    self.joint_map[name].publish(cmd_msg)

        except IndexError:
            # Sometimes joint_states messages are incomplete during startup
            pass

def main(args=None):
    rclpy.init(args=args)
    node = HardwareSplitter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()