import os
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Camera configurations
    cam_configs = {
        "cam0": {
            "camera_name": "cam0",
            "frame_id": "cam0_frame",
            "video_device": "/dev/video0",
        },
        "cam1": {
            "camera_name": "cam1",
            "frame_id": "cam1_frame",
            "video_device": "/dev/video2",
        },
        "cam2": {
            "camera_name": "cam2",
            "frame_id": "cam2_frame",
            "video_device": "/dev/video4",
        },
    }

    # Common settings
    shared_params = {
        "framerate": 30.0,
        "image_width": 1920,
        "image_height": 1080,
        "pixel_format": "mjpeg2rgb",  # Must be in supported list
        "io_method": "mmap",
        "brightness": 0,
        "contrast": 48,
    }

    nodes = []

    for cam_name, cam_info in cam_configs.items():
        params = {**shared_params, **cam_info}
        node = Node(
            package='usb_cam',
            executable='usb_cam_node_exe',
            name=cam_name,
            output='screen',
            parameters=[params],
            remappings=[
                ('image_raw', f'{cam_name}/image_raw'),
                ('image_raw/compressed', f'{cam_name}/image_compressed'),
                ('image_raw/compressedDepth', f'{cam_name}/compressedDepth'),
                ('image_raw/theora', f'{cam_name}/image_raw/theora'),
                ('camera_info', f'{cam_name}/camera_info')
            ]
        )
        nodes.append(node)

    return LaunchDescription(nodes)
