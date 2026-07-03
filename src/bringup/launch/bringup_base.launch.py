from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    declare_xacro_file_arg = DeclareLaunchArgument(
        'xacro_file',
        description='Path to the xacro file'
    )

    declare_use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    declare_localization_launch_arg = DeclareLaunchArgument(
        'localization_launch',
        default_value='localization.launch.py',
        description='Localization launch file to include'
    )

    use_sim_time = LaunchConfiguration('use_sim_time')
    localization_launch = LaunchConfiguration('localization_launch')

    description = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('description'),
                'launch',
                'description.launch.py'
            ])
        ),
        launch_arguments={
            'xacro_file':    LaunchConfiguration('xacro_file'),
            'use_sim_time':  use_sim_time,
        }.items()
    )

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('localization'),
                'launch',
                localization_launch
            ])
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    planning = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('planning'),
                'launch',
                'planning.launch.py'
            ])
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('perception'),
                'launch',
                'perception.launch.py'
            ])
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    lane_bev_carrot_node = Node(
        package='perception',
        executable='lane_bev_carrot',
        name='lane_bev_carrot',
        output='screen',
        parameters=[{
            'use_sim_time':          use_sim_time,
            'carrot_dist_m':         4.8,
            'goal_tolerance':        0.15,
            'publish_rate':          2.0,
            'camera_hfov':           1.047,
            'image_width':           640,
            'image_height':          480,
            'min_proj_m':            0.3,
            'max_proj_m':            6.0,
            'n_bev_samples':         50,
            'fit_cache_sec':         1.0,
            'no_carrot_stop_streak': 3,
            'safe_cost_max': 50,
            'safety_radius':         0.6,
            'max_carrot_dist_m': 6.0,
        }]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('bringup'),
            'config',
            'bringup.rviz'
        ])],
        output='screen'
    )

    return LaunchDescription([
        declare_xacro_file_arg,
        declare_use_sim_time_arg,
        declare_localization_launch_arg,
        description,
        localization,
        planning,
        perception,
        lane_bev_carrot_node,
        rviz_node,
    ])
