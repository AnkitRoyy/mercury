from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    pkg_rplidar = get_package_share_directory('rplidar_ros')
    rplidar_launch = os.path.join(pkg_rplidar, 'launch', 'rplidar_a3_launch.py')

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rplidar_launch),
        launch_arguments={
            'frame_id': 'laser'
        }.items()
    )

    video_device_arg   = DeclareLaunchArgument('video_device', default_value='/dev/video2')
    auto_exposure_arg  = DeclareLaunchArgument('auto_exposure', default_value='1')
    exposure_time_arg  = DeclareLaunchArgument('exposure_time_absolute', default_value='50')
    brightness_arg     = DeclareLaunchArgument('brightness', default_value='1')
    gain_arg           = DeclareLaunchArgument('gain', default_value='1')
    backlight_arg      = DeclareLaunchArgument('backlight_compensation', default_value='0')
    video_device       = LaunchConfiguration('video_device')

    usb_cam = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='usb_cam_node',
        output='screen',
        parameters=[{
            'video_device': video_device,
            'pixel_format': 'yuyv',
            'image_width': 640,
            'image_height': 480,
        }],
        remappings=[('image_raw', '/camera/image_raw')]
    )

    v4l2_ctrls = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(cmd=['v4l2-ctl', ['--device=', video_device], ['--set-ctrl=auto_exposure=', LaunchConfiguration('auto_exposure')]], output='screen'),
            ExecuteProcess(cmd=['v4l2-ctl', ['--device=', video_device], ['--set-ctrl=exposure_time_absolute=', LaunchConfiguration('exposure_time_absolute')]], output='screen'),
            ExecuteProcess(cmd=['v4l2-ctl', ['--device=', video_device], ['--set-ctrl=brightness=', LaunchConfiguration('brightness')]], output='screen'),
            ExecuteProcess(cmd=['v4l2-ctl', ['--device=', video_device], ['--set-ctrl=gain=', LaunchConfiguration('gain')]], output='screen'),
            ExecuteProcess(cmd=['v4l2-ctl', ['--device=', video_device], ['--set-ctrl=backlight_compensation=', LaunchConfiguration('backlight_compensation')]], output='screen'),
            ExecuteProcess(cmd=['v4l2-ctl', ['--device=', video_device], '--set-ctrl=power_line_frequency=1'], output='screen'),
        ]
    )

    # Teensy RPM converter for motor control via serial (micro USB)
    teensy = Node(
        package='hardware',
        executable='rpm_converter.py',
        name='teensy',
        output='screen',
        parameters=[{
            'wheel_radius': 0.075,
            'wheel_separation': 0.44,
            'max_wheel_rpm': 240.0,
            'serial_port': '/dev/ttyACM0',
            'serial_baud': 115200,
            'enable_serial': True,
            'enable_debug': True,
        }]
    )

    gps = Node(
        package='nmea_navsat_driver',
        executable='nmea_serial_driver',
        name='gps',
        output='screen',
        parameters=[{
            'port': '/dev/ttyACM1',
            'baud': 38400,
            'frame_id': 'gps_link',
        }],
        remappings=[('fix', '/gps')]
    )

    return LaunchDescription([
        # lidar,
        video_device_arg,
        auto_exposure_arg,
        exposure_time_arg,
        brightness_arg,
        gain_arg,
        backlight_arg,
        usb_cam,
        v4l2_ctrls,
        teensy,
        gps,  
    ])