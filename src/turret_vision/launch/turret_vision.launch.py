#!/usr/bin/env python3
"""
turret_vision.launch.py with working camera parameters
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description() -> LaunchDescription:
    
    # Camera lighting presets with ACTUALLY WORKING parameters
    camera_presets = {
        'super_cloudy': {
            'brightness': '64', 'contrast': '20', 'saturation': '80',
            'sharpness': '32', 'gain': '100', 'gamma': '200'
        },
        'cloudy': {
            'brightness': '50', 'contrast': '40', 'saturation': '80',
            'sharpness': '32', 'gain': '70', 'gamma': '180'
        },
        'partly_cloudy': {
            'brightness': '30', 'contrast': '50', 'saturation': '80',
            'sharpness': '32', 'gain': '40', 'gamma': '160'
        },
        'sunny': {
            'brightness': '10', 'contrast': '60', 'saturation': '80',
            'sharpness': '32', 'gain': '15', 'gamma': '140'
        },
        'very_bright': {
            'brightness': '-10', 'contrast': '70', 'saturation': '80',
            'sharpness': '32', 'gain': '5', 'gamma': '120'
        },
        'super_bright': {
            'brightness': '-30', 'contrast': '80', 'saturation': '80',
            'sharpness': '32', 'gain': '0', 'gamma': '100'
        }
    }
    
    # Arguments
    lighting_arg = DeclareLaunchArgument(
        'lighting_condition',
        default_value='partly_cloudy',
        description='Lighting condition: super_cloudy, cloudy, partly_cloudy, sunny, very_bright, super_bright'
    )
    
    target_image_arg = DeclareLaunchArgument(
        'target_image',
        default_value='/home/soap/probes/mercury/data/target.jpg',
        description='Path to target face image'
    )
    
    device_arg = DeclareLaunchArgument(
        'camera_device',
        default_value='/dev/video4',
        description='Camera device path'
    )
    
    # Function to get camera parameters based on lighting condition
    def get_camera_params(context, *args, **kwargs):
        lighting = context.launch_configurations.get('lighting_condition', 'partly_cloudy')
        params = camera_presets.get(lighting, camera_presets['partly_cloudy'])
        
        # Create camera node with selected parameters
        camera_node = Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name='usb_cam',
            output='screen',
            parameters=[
                {'video_device': LaunchConfiguration('camera_device').perform(context)},
                {'image_width': 640},
                {'image_height': 480},
                {'pixel_format': 'yuyv'},
                {'brightness': int(params['brightness'])},
                {'contrast': int(params['contrast'])},
                {'saturation': int(params['saturation'])},
                {'sharpness': int(params['sharpness'])},
                {'gain': int(params['gain'])},
                {'gamma': int(params['gamma'])},
            ],
            remappings=[
                ('/camera/image_raw', '/turret_camera/image_raw'),
            ],
        )
        
        recognition_node = Node(
            package='turret_vision',
            executable='recognition',
            name='recognition',
            output='screen',
            parameters=[{
                'target_image_path': LaunchConfiguration('target_image').perform(context),
                'similarity_threshold': 0.35,
            }],
            remappings=[
                ('/camera/image_raw', '/turret_camera/image_raw'),
            ],
        )
        
        return [camera_node, recognition_node]
    
    return LaunchDescription([
        lighting_arg,
        target_image_arg,
        device_arg,
        OpaqueFunction(function=get_camera_params),
    ])