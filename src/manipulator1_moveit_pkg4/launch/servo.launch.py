import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    # Load MoveIt config (URDF, SRDF, kinematics)
    moveit_config = (
        MoveItConfigsBuilder("manipulator1", package_name="manipulator1_moveit_pkg4")
        .robot_description(file_path="config/manipulator1.urdf.xacro")
        .robot_description_semantic(file_path="config/manipulator1.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        # .trajectory_execution(file_path="config/moveit_controllers.yaml")
        # .joint_limits(file_path="config/joint_limits.yaml")
        # .planning_scene_monitor(
        #     publish_robot_description = True,
        #     publish_robot_description_semantic = True
        # )
        .to_moveit_configs()
    )


    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        name="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    # Load Servo YAML
    pkg_path = get_package_share_directory("manipulator1_moveit_pkg4")
    with open(os.path.join(pkg_path, "config", "moveit_servo.yaml"), "r") as f:
        servo_yaml = yaml.safe_load(f)

    # 🔴 STRIP namespace (Servo expects private params)
    servo_params = {"moveit_servo": servo_yaml}
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        name="servo_node",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            {"planning_group_name": "Arm"},
            servo_params,   # ✅ correct scoping
        ],
    )

    return LaunchDescription([
        move_group_node,
        servo_node
    ])
