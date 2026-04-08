# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node

# from std_msgs.msg import Float64MultiArray, Float32


# class ServoToMotorBridge(Node):

#     def __init__(self):
#         super().__init__('servo_to_motor_bridge')

#         # -----------------------------
#         # Subscribers
#         # -----------------------------
#         self.create_subscription(
#             Float64MultiArray,
#             '/servo_velocity_controller/commands',
#             self.servo_cb,
#             10
#         )

#         # -----------------------------
#         # Publishers
#         # -----------------------------
#         self.base_pub = self.create_publisher(Float32, '/base/target', 10)
#         self.l1_pub   = self.create_publisher(Float32, '/l1/target',   10)
#         self.l2_pub   = self.create_publisher(Float32, '/l2/target',   10)

#         self.get_logger().info("Servo → Motor bridge started")





#     # -----------------------------
#     # Callback
#     # -----------------------------
#     def servo_cb(self, msg):

#         if len(msg.data) < 3:
#             self.get_logger().warn("Received less than 3 joint values")
#             return

#         base_val = float(msg.data[0])
#         l1_val   = float(msg.data[1])
#         l2_val   = float(msg.data[2])

#         base_msg = Float32()
#         l1_msg   = Float32()
#         l2_msg   = Float32()

#         base_msg.data = base_val
#         l1_msg.data   = l1_val
#         l2_msg.data   = l2_val

#         self.base_pub.publish(base_msg)
#         self.l1_pub.publish(l1_msg)
#         self.l2_pub.publish(l2_msg)


# def main():
#     rclpy.init()
#     node = ServoToMotorBridge()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()
    
    
    
    
    
    
    
    
    
    
    
    
    
    
# #!/usr/bin/env python3

# import rclpy
# from rclpy.node import Node

# from std_msgs.msg import Float64MultiArray, Float32

# from rclpy.qos import QoSProfile, ReliabilityPolicy

# qos_best_effort = QoSProfile(
#     depth=10,
#     reliability=ReliabilityPolicy.BEST_EFFORT
# )





# class ServoToMotorBridge(Node):

#     def __init__(self):
#         super().__init__('servo_to_motor_bridge')

#         # -----------------------------
#         # Subscribers
#         # -----------------------------
#         self.create_subscription(
#             Float64MultiArray,
#             '/servo_velocity_controller/commands',
#             self.servo_cb,
#             10
#         )

        

#         # -----------------------------
#         # Publishers
#         # -----------------------------
#         self.base_pub = self.create_publisher(Float32, '/base/target', qos_best_effort)
#         self.l1_pub   = self.create_publisher(Float32, '/l1/target',   qos_best_effort)
#         self.l2_pub   = self.create_publisher(Float32, '/l2/target',   qos_best_effort)

#         # -----------------------------
#         # Storage for latest values
#         # -----------------------------
#         self.base_val = 0.0
#         self.l1_val   = 0.0
#         self.l2_val   = 0.0
#         self.have_data = False

#         # -----------------------------
#         # 10 Hz publish timer
#         # -----------------------------
#         publish_rate_hz = 20.0
#         period = 1.0 / publish_rate_hz
#         self.timer = self.create_timer(period, self.publish_targets)

#         self.get_logger().info("Servo → Motor bridge started (Publishing at 10 Hz)")

#     # -----------------------------
#     # Subscriber Callback
#     # -----------------------------
#     def servo_cb(self, msg):

#         if len(msg.data) < 3:
#             self.get_logger().warn("Received less than 3 joint values")
#             return

#         self.base_val = float(msg.data[0])
#         self.l1_val   = float(msg.data[1])
#         self.l2_val   = float(msg.data[2])
#         self.have_data = True

#     # -----------------------------
#     # Timer Publish Callback (10 Hz)
#     # -----------------------------
#     def publish_targets(self):

#         # Don't publish until first message arrives
#         if not self.have_data:
#             return

#         base_msg = Float32()
#         l1_msg   = Float32()
#         l2_msg   = Float32()

#         base_msg.data = self.base_val
#         l1_msg.data   = self.l1_val
#         l2_msg.data   = self.l2_val

#         self.base_pub.publish(base_msg)
#         self.l1_pub.publish(l1_msg)
#         self.l2_pub.publish(l2_msg)


# def main():
#     rclpy.init()
#     node = ServoToMotorBridge()
#     rclpy.spin(node)
#     node.destroy_node()
#     rclpy.shutdown()


# if __name__ == '__main__':
#     main()

    
    
    
    





#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray, Float32
from rclpy.qos import QoSProfile, ReliabilityPolicy

import math

# -----------------------------
# BEST EFFORT QoS (important for ESP32)
# -----------------------------
qos_best_effort = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT
)

class ServoToMotorBridge(Node):

    def __init__(self):
        super().__init__('servo_to_motor_bridge')

        # -----------------------------
        # Subscriber
        # -----------------------------
        self.create_subscription(
            Float64MultiArray,
            '/servo_velocity_controller/commands',
            self.servo_cb,
            10
        )

        # -----------------------------
        # Publishers (BEST_EFFORT)
        # -----------------------------
        self.base_pub = self.create_publisher(Float32, '/base/target', qos_best_effort)
        self.l1_pub   = self.create_publisher(Float32, '/l1/target',   qos_best_effort)
        self.l2_pub   = self.create_publisher(Float32, '/l2/target',   qos_best_effort)

        # -----------------------------
        # Storage for latest values (DEGREES)
        # -----------------------------
        self.base_deg = 0.0
        self.l1_deg   = 0.0
        self.l2_deg   = 0.0
        self.have_data = False

        # -----------------------------
        # Publish timer (10 Hz)
        # -----------------------------
        publish_rate_hz = 50.0
        period = 1.0 / publish_rate_hz
        self.timer = self.create_timer(period, self.publish_targets)

        self.get_logger().info("Servo → Motor bridge started (Publishing DEGREES at 10 Hz)")

    # -----------------------------
    # Subscriber Callback
    # -----------------------------
    def servo_cb(self, msg):

        if len(msg.data) < 3:
            self.get_logger().warn("Received less than 3 joint values")
            return

        # Incoming values are in RADIANS
        base_rad = float(msg.data[0])
        l1_rad   = float(msg.data[1])
        l2_rad   = float(msg.data[2])

        # Convert to DEGREES
        self.base_deg = math.degrees(base_rad)
        self.l1_deg   = math.degrees(l1_rad)
        self.l2_deg   = math.degrees(l2_rad)

        self.have_data = True

    # -----------------------------
    # Timer Publish Callback (10 Hz)
    # -----------------------------
    def publish_targets(self):

        # Don't publish until first message arrives
        if not self.have_data:
            return

        base_msg = Float32()
        l1_msg   = Float32()
        l2_msg   = Float32()

        # Publishing DEGREES
        base_msg.data = self.base_deg
        l1_msg.data   = self.l1_deg
        l2_msg.data   = self.l2_deg

        self.base_pub.publish(base_msg)
        self.l1_pub.publish(l1_msg)
        self.l2_pub.publish(l2_msg)


def main():
    rclpy.init()
    node = ServoToMotorBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()