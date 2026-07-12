from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# ─── Serial Port Defaults ────────────────────────────────────────────────────

LIDAR_PORT  = '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_47bd82635cda7e44a9f4fb6e59533130-if00-port0'
TEENSY_PORT = '/dev/serial/by-id/usb-Teensyduino_USB_Serial_19904990-if00'
GPS_PORT    = '/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00'

# To test higher res later:  pixel_format='mjpeg2rgb', 1280x720
# Alternative package:       ros-jazzy-v4l2-camera (handles MJPEG correctly)
#
# Angetube v4l2 control ranges:
#   auto_exposure            : 1=manual, 3=auto  (default=3)  ← set PRE-stream
#   exposure_time_absolute   : 1–5000 (100µs)    (default=100)← set PRE-stream
#   focus_automatic_continuous: bool             (default=1)  ← set PRE-stream
#   brightness               : 1–64              (default=30)
#   gain                     : 0–15              (default=0)
#   backlight_compensation   : 0–7               (default=0)
#   power_line_frequency     : 0=off,1=50Hz,2=60Hz (default=1)

CAM_DEVICE  = '/dev/video3'
CAM_WIDTH   = 640
CAM_HEIGHT  = 480
CAM_FPS     = 30
CAM_FORMAT  = 'yuyv2rgb'    # stable on Jazzy; switch to mjpeg2rgb once bug is resolved

PRE_CTRLS = [
    'auto_exposure=0',              # 1=manual (unlocks exposure_time_absolute)
    'exposure_time_absolute=4',   # 100µs units; 150=15ms — tuned for indoor/outdoor
    'focus_automatic_continuous=0', # disable autofocus hunt
]

POST_CTRLS = [
    'brightness=0',                # range 1–64
    'gain=0',                       # range 0–15
    'backlight_compensation=0',     # 0=off
    'power_line_frequency=1',       # 1=50Hz (India)
]


def generate_launch_description():


    video_device_arg = DeclareLaunchArgument('video_device', default_value=CAM_DEVICE)
    video_device     = LaunchConfiguration('video_device')


    pre_cam_setup = ExecuteProcess(
        cmd=['v4l2-ctl', ['--device=', video_device]] +
            [f'--set-ctrl={c}' for c in PRE_CTRLS],
        output='screen',
        name='pre_cam_setup',
    )


    usb_cam = TimerAction(
        period=1.0,
        actions=[Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam_node',
            output='screen',
            parameters=[{
                'video_device': video_device,
                'pixel_format': CAM_FORMAT,
                'image_width':  CAM_WIDTH,
                'image_height': CAM_HEIGHT,
                'framerate':    float(CAM_FPS),
                'brightness':   32,
                'gain':         0,
            }],
            remappings=[('image_raw', '/camera/image_raw')],
        )],
    )

    post_cam_ctrls = TimerAction(
        period=3.0,
        actions=[ExecuteProcess(
            cmd=['v4l2-ctl', ['--device=', video_device]] +
                [f'--set-ctrl={c}' for c in POST_CTRLS],
            output='screen',
            name='post_cam_ctrls',
        )],
    )

    lidar_serial_port_arg  = DeclareLaunchArgument('lidar_serial_port',  default_value=LIDAR_PORT)
    teensy_serial_port_arg = DeclareLaunchArgument('teensy_serial_port', default_value=TEENSY_PORT)
    gps_serial_port_arg    = DeclareLaunchArgument('gps_serial_port',    default_value=GPS_PORT)

    lidar = Node(
        package='rplidar_ros',
        executable='rplidar_composition',
        name='rplidar_composition',
        output='screen',
        parameters=[{
            'serial_port':      LaunchConfiguration('lidar_serial_port'),
            'serial_baudrate':  256000,
            'frame_id':         'laser_link',
            'inverted':         False,
            'angle_compensate': True,
            'scan_mode':        'Standard',
        }],
    )

    teensy = Node(
        package='hardware',
        executable='rpm_converter.py',
        name='teensy',
        output='screen',
        parameters=[{
            'wheel_radius':     0.24,
            'wheel_separation': 0.84,
            'max_wheel_rpm':    240.0,
            'serial_port':      LaunchConfiguration('teensy_serial_port'),
            'serial_baud':      115200,
            'enable_serial':    True,
            'enable_debug':     True,
        }],
    )

    gps = Node(
        package='nmea_navsat_driver',
        executable='nmea_serial_driver',
        name='gps',
        output='screen',
        parameters=[{
            'port':     LaunchConfiguration('gps_serial_port'),
            'baud':     38400,
            'frame_id': 'gps_link',
        }],
        remappings=[('fix', '/gps')],
    )


    return LaunchDescription([
        video_device_arg,
        pre_cam_setup,      
        usb_cam,            
        post_cam_ctrls,     
        lidar_serial_port_arg, lidar,
        teensy_serial_port_arg, teensy,
        gps_serial_port_arg, gps,
    ])