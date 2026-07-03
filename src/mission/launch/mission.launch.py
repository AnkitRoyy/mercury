from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import (
    PythonLaunchDescriptionSource,
)

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

from ament_index_python.packages import get_package_share_directory
import os

GENERATED_YAML = os.path.join(
    get_package_share_directory("mission"), "config", "mission_params.yaml"
)

def generate_launch_description():

    mission_setup = Node(
        package="mission",
        executable="mission_setup",
        name="mission_setup",
        output="screen",
        parameters=[{
            "generated_yaml_path":
                GENERATED_YAML
        }]
    )

    watchdog = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("watchdog_monitor"),
                "launch",
                "watchdog.launch.py"
            ])
        ),
        launch_arguments={
            "params_file":
                GENERATED_YAML
        }.items()
    )

    turret = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("turret_vision"),
                "launch",
                "turret_vision.launch.py"
            ])
        )
    )

    waypoint_sender = Node(
        package="mission",
        executable="waypoint_sender",
        name="waypoint_sender",
        output="screen",
        parameters=[{
            "generated_yaml_path":
                GENERATED_YAML
        }]
    )

    start_mission_stack = RegisterEventHandler(
        OnProcessExit(
            target_action=mission_setup,
            on_exit=[
                watchdog,
                turret,
                waypoint_sender,
            ]
        )
    )

    return LaunchDescription([
        mission_setup,
        start_mission_stack,
    ])