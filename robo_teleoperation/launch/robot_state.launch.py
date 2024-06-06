from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("melfa_assista").to_moveit_configs()

    # MoveGroupInterface demo executable
    node = Node(
        name="teleop_controller_node",
        package="robo_teleoperation",
        executable="teleop_controller_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription([node])