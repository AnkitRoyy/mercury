from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# ─── Serial Port Defaults ────────────────────────────────────────────────────

LIDAR_PORT  = '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_47bd82635cda7e44a9f4fb6e59533130-if00-port0'
TEENSY_PORT = '/dev/serial/by-id/usb-Teensyduino_USB_Serial_19085340-if00'
GPS_PORT    = '/dev/serial/by-id/usb-u-blox_AG_-_www.u-blox.com_u-blox_GNSS_receiver-if00'

# ─── Camera Config ────────────────────────────────────────────────────────────
# usb_cam Jazzy has a buffer-overflow segfault in its mjpeg2rgb decoder at
# resolutions above 640x480. Use yuyv2rgb (stable) until that's patched or
# we switch to the v4l2_camera package.
#
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

CAM_DEVICE  = '/dev/video2'
CAM_WIDTH   = 640
CAM_HEIGHT  = 480
CAM_FPS     = 30
CAM_FORMAT  = 'yuyv2rgb'    # stable on Jazzy; switch to mjpeg2rgb once bug is resolved

# Volatile controls — MUST be set before usb_cam opens the device.
# Changing auto_exposure while streaming causes SIGSEGV in the UVC kernel driver.
PRE_CTRLS = [
    'auto_exposure=1',              # 1=manual (unlocks exposure_time_absolute)
    'exposure_time_absolute=15',   # 100µs units; 150=15ms — tuned for indoor/outdoor
    'focus_automatic_continuous=0', # disable autofocus hunt
]

# Non-volatile controls — safe to apply after streaming starts.
POST_CTRLS = [
    'brightness=0',                # range 1–64
    'gain=0',                       # range 0–15
    'backlight_compensation=0',     # 0=off
    'power_line_frequency=1',       # 1=50Hz (India)
]


def generate_launch_description():

    # ── Args ─────────────────────────────────────────────────────────────────

    video_device_arg = DeclareLaunchArgument('video_device', default_value=CAM_DEVICE)
    video_device     = LaunchConfiguration('video_device')

    # ── Phase 1 (t=0s): set volatile controls before device opens ─────────────
    # Runs a single v4l2-ctl call with all PRE_CTRLS to avoid multiple processes.

    pre_cam_setup = ExecuteProcess(
        cmd=['v4l2-ctl', ['--device=', video_device]] +
            [f'--set-ctrl={c}' for c in PRE_CTRLS],
        output='screen',
        name='pre_cam_setup',
    )

    # ── Phase 2 (t=1s): launch usb_cam ────────────────────────────────────────
    # Device opens into a stable manual-exposure state set above.
    # 3 "unknown control" warnings (white_balance_temperature_auto, exposure_auto,
    # focus_auto) are hardcoded in usb_cam's C++ — different UVC name mapping than
    # this camera's v4l2 names. They are harmless and cannot be suppressed without
    # patching usb_cam source.

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

    # ── Phase 3 (t=3s): apply non-volatile tweaks ────────────────────────────
    # Single v4l2-ctl call — cleaner than spawning 4 separate processes.

    post_cam_ctrls = TimerAction(
        period=3.0,
        actions=[ExecuteProcess(
            cmd=['v4l2-ctl', ['--device=', video_device]] +
                [f'--set-ctrl={c}' for c in POST_CTRLS],
            output='screen',
            name='post_cam_ctrls',
        )],
    )

    # ── Hardware nodes (uncomment to enable) ─────────────────────────────────

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
            'wheel_radius':     0.075,
            'wheel_separation': 0.44,
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

    # ── Launch description ────────────────────────────────────────────────────

    return LaunchDescription([
        # Camera (3-phase startup to prevent SIGSEGV)
        video_device_arg,
        pre_cam_setup,      # t=0s  volatile controls before device opens
        usb_cam,            # t=1s  device opens into stable state
        post_cam_ctrls,     # t=3s  non-volatile tweaks while streaming
        lidar_serial_port_arg, lidar,
        # teensy_serial_port_arg, teensy,
        # gps_serial_port_arg, gps,
    ])