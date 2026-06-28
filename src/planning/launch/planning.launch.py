from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    pkg_planning = get_package_share_directory('planning')
    use_sim_time = LaunchConfiguration('use_sim_time')

    params = os.path.join(pkg_planning, 'config', 'nav2_params.yaml')
    global_costmap = os.path.join(pkg_planning, 'config', 'global_costmap.yaml')
    local_costmap = os.path.join(pkg_planning, 'config', 'local_costmap.yaml')
    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params,
            param_rewrites={'use_sim_time': use_sim_time},
            convert_types=True,
        ),
        allow_substs=True,
    )
    configured_global_costmap = ParameterFile(
        RewrittenYaml(
            source_file=global_costmap,
            param_rewrites={'use_sim_time': use_sim_time},
            convert_types=True,
        ),
        allow_substs=True,
    )
    configured_local_costmap = ParameterFile(
        RewrittenYaml(
            source_file=local_costmap,
            param_rewrites={'use_sim_time': use_sim_time},
            convert_types=True,
        ),
        allow_substs=True,
    )

    planner = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        parameters=[configured_params, configured_global_costmap],
        output='screen'
    )

    controller = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        parameters=[configured_params, configured_local_costmap],
        output='screen'
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        parameters=[configured_params],
        output='screen'
    )

    behavior = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        parameters=[configured_params],
        output='screen'
    )

    lifecycle_manager = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': True,
            'node_names': [
                'planner_server',
                'controller_server',
                'bt_navigator',
                'behavior_server'
            ]
        }]
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation (Gazebo) clock if true'
        ),
        planner,
        controller,
        bt_navigator,
        behavior,
        lifecycle_manager
    ])
