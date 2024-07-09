import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    # Path to the config file
    config_file_path = os.path.join(
        get_package_share_directory('planning_scene_spawner'),
        'config',
        'scene.yaml'
    )

    return LaunchDescription([
        # Declare the launch arguments
        DeclareLaunchArgument(
            'config_file',
            default_value=config_file_path,
            description='Full path to the YAML file to load parameters from'
        ),
        # Node configuration
        Node(
            package='planning_scene_spawner',
            executable='planning_scene_spawner_node',
            name='planning_scene_spawner',
            output='screen',
            parameters=[LaunchConfiguration('config_file')]
        )
    ])

