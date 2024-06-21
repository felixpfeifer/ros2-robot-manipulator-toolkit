from moveit_configs_utils import MoveItConfigsBuilder
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("robot", package_name="melfa_assista_moveit_config")
        .robot_description(file_path="config/robot.urdf.xacro")
        .robot_description_semantic(file_path="config/robot.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    # Define the IP argument with a new default value
    new_ip_argument = DeclareLaunchArgument(
        'ip',
        default_value='141.7.42.138',
        description="IP for the Sensopart Camera"
    )

    # Pass the argument to the included launch file
    sensopart_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('sensopart_connector'),
            '/sensopart_connector.launch.py'
        ]),
        launch_arguments={'ip': LaunchConfiguration('ip')}.items()
    )

    return LaunchDescription(
        [
            Node(
                package="sensopart_vision_ros2",
                executable="sensopart_vision_node",
                name="sensopart_vision_node",
                output="screen",
                parameters=[moveit_config.to_dict()],
            ),
            new_ip_argument,
            sensopart_launch,
        ]
    )
