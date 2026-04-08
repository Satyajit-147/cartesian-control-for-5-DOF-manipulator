import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_pkg_name = "manipulator1_moveit_servo2"  # Make sure this matches your package name!

    # 1. Build MoveIt Configs
    moveit_config = MoveItConfigsBuilder("manipulator1", package_name=moveit_pkg_name).to_moveit_configs()

    # 2. Get the Servo Config File
    servo_yaml_path = os.path.join(
        get_package_share_directory(moveit_pkg_name),
        "config",
        "moveit_servo.yaml"
    )
    
    # Load the YAML file
    with open(servo_yaml_path, 'r') as f:
        servo_params = yaml.safe_load(f)

    # 3. Servo Node 
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        output="screen",
        parameters=[
            servo_params,  # Load the file we just read
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    # 4. ROS2 Control Node
    ros2_controllers_path = os.path.join(
        get_package_share_directory(moveit_pkg_name),
        "config",
        "ros2_controllers.yaml"
    )
    
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="screen",
    )

    # 5. Spawners
    # Spawns the VELOCITY Controller for Servo
    servo_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["servo_velocity_controller", "--controller-manager", "/controller_manager"],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )

    # 6. Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # 7. RViz
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", os.path.join(get_package_share_directory(moveit_pkg_name), "config", "moveit.rviz")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription([
        ros2_control_node,
        robot_state_publisher,
        joint_state_broadcaster_spawner,
        servo_controller_spawner,
        servo_node,
        rviz_node
    ])