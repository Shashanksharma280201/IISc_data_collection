import os
import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import LifecycleNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import yaml

def generate_launch_description():
    cam_names = ["cam0", "cam1"]
    config = {}

    config["cam0"] = {
        "camera_name": "cam0",
        "framerate": 30.0,
        "frame_id": "cam0",
        "image_width": 1920,
        "image_height": 1080,
        "video_device": "/dev/video4",
        "pixel_format": "uyvy",  # <- Try this instead of mjpeg2rgb
        "io_method": "mmap",
        "brightness": 0,
        "contrast": 48
    }


    config["cam1"] = {
        "camera_name": "cam1",
        "framerate": 30.0,
        "frame_id": "cam1",
        "image_width": 1920,
        "image_height": 1080,
        "video_device": "/dev/video2",
        "pixel_format": "uyvy",  # <- Try this instead of mjpeg2rgb
        "io_method": "mmap",
        "brightness": 0,
        "contrast": 48
    }
    
    config["cam2"] = {
        "camera_name": "cam2",
        "framerate": 60.0,
        "frame_id": "cam0",
        "image_width": 1920,
        "image_height": 1080,
        "video_device": "/dev/video0",
        "pixel_format": "uyvy",  # <- Try this instead of mjpeg2rgb
        "io_method": "mmap",
        "brightness": 0,
        "contrast": 48
    }

    nodes = []

    for cam in cam_names:
        usb_cam = Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name=cam,
            output='screen',
            parameters=[config[cam]],
            remappings=[
                ('image_raw', f'{cam}/image_raw'),
                ('image_raw/compressed', f'{cam}/image_compressed'),
                ('image_raw/compressedDepth', f'{cam}/compressedDepth'),
                ('image_raw/theora', f'{cam}/image_raw/theora'),
                ('camera_info', f'{cam}/camera_info')
            ]
        )
        nodes.append(usb_cam)




    

    return LaunchDescription(nodes)
