import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 1. Setup the package and file names
    package_name = "manipulator1_moveit_pkg4"
    servo_config_file = "servo_config.yaml"

    # 2. Build MoveIt configs 
    # This automatically finds your URDF, SRDF, and kinematics.yaml 
    # within the specified package.
    moveit_config = MoveItConfigsBuilder("manipulator1", package_name=package_name).to_moveit_configs()

    # 3. Get the path to your Servo config file
    # This assumes the file is located in the 'config' folder of your package
    servo_yaml_path = os.path.join(
        get_package_share_directory(package_name),
        "config",
        servo_config_file
    )
    
    # Load the YAML content into a dictionary
    servo_params = {"moveit_servo": servo_yaml_path}

    # 4. Define the Servo Node
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
        output="screen",
    )

    return LaunchDescription([servo_node])