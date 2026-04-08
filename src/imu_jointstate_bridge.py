#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
import math

from rclpy.qos import QoSProfile, ReliabilityPolicy


# -----------------------------
# BEST EFFORT QoS (important for ESP32)
# -----------------------------
qos_best_effort = QoSProfile(
    depth=10,
    reliability=ReliabilityPolicy.BEST_EFFORT
)


class AngleToJointState(Node):

    def __init__(self):
        super().__init__('angle_to_jointstate')

        # -----------------------------
        # Joint names (must match URDF)
        # -----------------------------
        self.joint_names = [
            'base_link_j_base_plate',
            'side_plate1_j_side_plate12',
            'motor_holder_j_side_plate_12'
        ]

        # Latest angles storage (DEGREES)
        self.base_angle_deg = 0.0
        self.l1_angle_deg   = 0.0
        self.l2_angle_deg   = 0.0

        # -----------------------------
        # Subscribers
        # -----------------------------
        self.create_subscription(
            Float32,
            '/base/angle',
            self.base_cb,
            qos_best_effort
        )

        self.create_subscription(
            Float32,
            '/l1/angle',
            self.l1_cb,
            qos_best_effort
        )

        self.create_subscription(
            Float32,
            '/l2/angle',
            self.l2_cb,
            qos_best_effort
        )

        # -----------------------------
        # Publisher
        # -----------------------------
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )

        # Publish at fixed rate (50 Hz is good for Servo)
        self.timer = self.create_timer(0.02, self.publish_joint_state)

        self.get_logger().info("Angle(deg) → /joint_states(rad) bridge started")

    # -----------------------------
    # Callbacks (store degrees)
    # -----------------------------
    def base_cb(self, msg):
        self.base_angle_deg = msg.data

    def l1_cb(self, msg):
        self.l1_angle_deg = msg.data

    def l2_cb(self, msg):
        self.l2_angle_deg = msg.data

    # -----------------------------
    # Publisher loop
    # -----------------------------
    def publish_joint_state(self):
        js = JointState()

        js.header.stamp = self.get_clock().now().to_msg()
        js.header.frame_id = 'base_link'
        js.name = self.joint_names

        # -----------------------------
        # Convert degrees → radians
        # -----------------------------
        base_rad = math.radians(self.base_angle_deg)
        l1_rad   = math.radians(self.l1_angle_deg)
        l2_rad   = math.radians(self.l2_angle_deg)

        # IMPORTANT: order exactly as requested
        js.position = [
            base_rad,
            l1_rad,
            l2_rad
        ]

        # Optional but recommended
        js.velocity = [0.0, 0.0, 0.0]
        js.effort   = [float('nan'), float('nan'), float('nan')]

        self.joint_state_pub.publish(js)


def main():
    rclpy.init()
    node = AngleToJointState()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()