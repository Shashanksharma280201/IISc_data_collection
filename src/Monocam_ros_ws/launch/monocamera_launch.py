import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package directory
    package_dir = get_package_share_directory('monocamera_publisher')
    
    # Camera configurations
    cam_configs = {
        "cam0": {
            "camera_name": "cam1",
            "frame_id": "cam1_frame",
            "video_device": "/dev/video2",
            "pose": ['0.2', '0.1', '0.0', '0', '0', '0', '1']  # example pose from lidar_imu_link
        },
        "cam1": {
            "camera_name": "cam2",
            "frame_id": "cam2_frame",
            "video_device": "/dev/video4",
            "pose": ['-0.2', '0.1', '0.0', '0', '0', '0', '1']  # example pose from lidar_imu_link
        },
    }

    shared_params = {
        "framerate": 30.0,
        "image_width": 1920,
        "image_height": 1080,
        "pixel_format": "mjpeg2rgb",
        "io_method": "mmap",
        "brightness": 0,
        "contrast": 48,
    }

    nodes = []

    # Camera nodes
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

    # Static TFs from lidar_imu_link → each camera frame
    for cam_name, cam_info in cam_configs.items():
        static_tf = Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'{cam_name}_tf_broadcaster',
            arguments=cam_info['pose'] + ['lidar_imu_link', cam_info['frame_id']],
            output='screen'
        )
        nodes.append(static_tf)

    # Camera info publishers
    for cam_name, cam_info in cam_configs.items():
        camera_info_pub = Node(
            package='monocamera_publisher',
            executable='camera_info_publisher.py',
            name=f'{cam_name}_info_publisher',
            output='screen',
            parameters=[{
                'camera_name': cam_info['camera_name'],
                'image_width': shared_params['image_width'],
                'image_height': shared_params['image_height'],
                'frame_id': cam_info['frame_id']
            }]
        )
        nodes.append(camera_info_pub)
    
    # RViz2 node
    rviz_config_file = os.path.join(package_dir, 'config', 'multicamera_rviz.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    nodes.append(rviz_node)
    
    return LaunchDescription(nodes)
