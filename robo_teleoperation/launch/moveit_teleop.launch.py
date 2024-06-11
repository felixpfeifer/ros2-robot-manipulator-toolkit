from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("robot",package_name="melfa_assista_moviet")
        .robot_description(file_path="config/robot.urdf.xacro")
        .robot_description_semantic(file_path="config/robot.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    return LaunchDescription(
        [
            Node(
                package="robo_teleoperation",
                executable="teleop_controller_node",
                name="teleop_controller_node",
                output="screen",
                parameters=[moveit_config.to_dict()],
            )
        ]
    )