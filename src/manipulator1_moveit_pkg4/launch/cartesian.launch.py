import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:
        return None

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def generate_launch_description():
    # 1. Setup MoveIt Config
    moveit_config = (
    MoveItConfigsBuilder("manipulator1", package_name="manipulator1_moveit_pkg")
        .robot_description(file_path="config/manipulator1.urdf.xacro")
        .robot_description_semantic(file_path="config/manipulator1.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .to_moveit_configs()
    )

    # 2. Get parameters for the Servo node
    servo_yaml = load_yaml("manipulator1_moveit_pkg4", "config/moveit_servo.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    # 3. RViz Node
    rviz_config_file = (
        get_package_share_directory("moveit_servo") + "/config/cartesian_rviz.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        #arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
    )

    # 4. ROS2 Control Node (Hardware Interface)
    ros2_controllers_path = os.path.join(
        get_package_share_directory("manipulator1_moveit_pkg4"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="screen",
    )

    # 5. Controller Spawners
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout",
            "300",
            "--controller-manager",
            "/controller_manager",
        ],
    )

    manipulator1_velocity_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["manipulator1_velocity_controller", "-c", "/controller_manager"],
    )


    manipulator1_gripper_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["Gripper_controller", "-c", "/controller_manager"],
    )


    # 6. Robot State Publisher (Converted from ComposableNode)
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[moveit_config.robot_description],
    )

    # 7. TF2 Static Transform (Converted from ComposableNode)
    # Note: Standalone static_transform_publisher uses arguments, not parameters
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf2_broadcaster",
        output="screen",
        arguments=["--frame-id", "world", "--child-frame-id", "base_link"],
    )

    # 8. Joy Node (Converted from ComposableNode)
    # joy_node = Node(
    #     package="joy",
    #     executable="joy_node",
    #     name="joy_node",
    #     output="screen",
    #     parameters=[{
    #         "autorepeat_rate": 50.0  # DEBUG  (increased now)
    #     }]
    # )

    # # 9. Joy to Servo Container
    # joy_to_twist_node = Node(
    #     package="manipulator1_moveit_pkg4",
    #     executable="joy_to_twist_node",  # Matches CMakeLists add_executable name
    #     name="joy_to_twist_custom",
    #     output="screen",
    #     parameters=[
    #         {"frame_id": "base_link"},         # Explicitly set frame
    #         {"scale_linear": 0.5},             # Set speed
    #         {"scale_angular": 1.0}             # Set rotation speed
    #     ]
    # )

    planning_group_name = {"planning_group_name": "Arm"}


    # 10. Servo Node (Main MoveIt Servo Logic)
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            servo_params,
            planning_group_name,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
        output="screen",
        #arguments=["--ros-args", "--log-level", "debug"], # DEBUG
    )

    return LaunchDescription(
        [
            rviz_node,
            ros2_control_node,
            joint_state_broadcaster_spawner,
            manipulator1_velocity_controller_spawner,
            manipulator1_gripper_controller_spawner,
            robot_state_publisher_node,
            static_tf_node,
            # joy_node,
            # joy_to_twist_node,

            servo_node,
        ]
    )