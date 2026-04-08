import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    moveit_pkg_name = "manipulator1_moveit_servo"

    # 1. Build MoveIt Configs
    moveit_config = MoveItConfigsBuilder("manipulator1", package_name=moveit_pkg_name).to_moveit_configs()

    # 2. Servo Node (Minimal Config)
    # We rely on the defaults for the group name since we renamed it to 'panda_arm'
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            # We explicitly set the topics, but leave the group name to default
            {"cartesian_command_in_topic": "/servo_node/delta_twist_cmds"},
            {"joint_command_in_topic": "/servo_node/delta_joint_cmds"},
            {"command_out_topic": "/servo_velocity_controller/commands"},
            {"command_out_type": "std_msgs/Float64MultiArray"},
            {"publish_period": 0.01},
            {"check_collisions": False},
            {"scale.linear": 0.4},
            {"scale.rotational": 0.4},
            {"scale.joint": 0.5},
        ],
    )

    # 3. Fake Hardware / ROS2 Control
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

    # 4. Spawners
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

    # 5. Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # 6. RViz
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