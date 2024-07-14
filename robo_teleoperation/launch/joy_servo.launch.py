import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
import yaml
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("robot", package_name="melfa_assista_moveit_config")
        .robot_description(file_path="config/robot.urdf.xacro")
        .to_moveit_configs()
    )

    # Get parameters for the Servo node
    servo_yaml = load_yaml("melfa_assista_moveit_config", "config/servo_config.yaml")

    # Set the speed limits in the Servo parameters
    servo_yaml["moveit_servo"]["command_in_topic"]["max_linear_velocity"] = 0.25  # 250 mm/s
    servo_yaml["moveit_servo"]["command_in_topic"]["max_angular_velocity"] = 0.25  # This value may need adjusting
    # based on specific requirements

    servo_params = {"moveit_servo": servo_yaml}

    if servo_yaml is None:
        raise RuntimeError("Failed to load servo YAML file")

    # Launch a standalone Servo node with intra-process communication enabled.
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

    # Launch the Xbox controller node
    xbox_controller_node = Node(
        package="robo_teleoperation",
        executable="xbox_controller",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
        ],
        output="screen",
    )

    # Launch the joy node
    joy_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
    )


    return LaunchDescription(
        [
            servo_node,
            joy_node,
            xbox_controller_node,
        ]
    )

if __name__ == '__main__':
    generate_launch_description()
