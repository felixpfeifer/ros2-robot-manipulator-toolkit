from launch import LaunchDescription
from launch_ros import event_handlers
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            LogInfo, RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                   OnProcessIO, OnProcessStart, OnShutdown)
from launch.events import Shutdown
from launch.substitutions import (EnvironmentVariable, FindExecutable,
                                  LaunchConfiguration, LocalSubstitution,
                                  PythonExpression)


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("robot", package_name="melfa_assista_moveit_config")
        .robot_description(file_path="config/robot.urdf.xacro")
        .robot_description_semantic(file_path="config/robot.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_pipelines(pipelines=["ompl", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    # Spawn the teleop_controller_node after the MoveGroup node is running
    teleop_node = Node(
        package="robot_teleoperation",
        executable="teleop_controller_node",
        name="teleop_controller_node",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    return LaunchDescription(
        [
            teleop_node
        ]
    )
