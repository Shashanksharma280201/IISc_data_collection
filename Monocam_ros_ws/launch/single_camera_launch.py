import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package directory
    package_dir = get_package_share_directory('monocamera_publisher')
    
    # Single camera configuration - Based on your working script
    camera_config = {
        "camera_name": "mono_camera",
        "frame_id": "camera_frame",
        "video_device": "/dev/video0",
        "framerate": 30.0,
        "image_width": 1920,
        "image_height": 1080,
        "pixel_format": "mjpeg2rgb",  # Must be in supported list
        "io_method": "mmap",
        "brightness": 0,
        "contrast": 48,
    }

    nodes = []

    # Create camera node
    camera_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='mono_camera',
        output='screen',
        parameters=[camera_config],
        remappings=[
            ('image_raw', 'mono_camera/image_raw'),
            ('image_raw/compressed', 'mono_camera/image_compressed'),
            ('image_raw/compressedDepth', 'mono_camera/compressedDepth'),
            ('image_raw/theora', 'mono_camera/image_raw/theora'),
            ('camera_info', 'mono_camera/camera_info')
        ]
    )
    nodes.append(camera_node)

    # Static transform publisher for camera frame
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_tf_broadcaster',
        arguments=[
            '0', '0', '0',  # x, y, z
            '0', '0', '0', '1',  # qx, qy, qz, qw (quaternion)
            'base_link',  # parent frame
            'camera_frame'  # child frame
        ]
    )
    nodes.append(static_tf)

    # Camera info publisher
    camera_info_pub = Node(
        package='monocamera_publisher',
        executable='camera_info_publisher.py',
        name='camera_info_publisher',
        output='screen',
        parameters=[{
            'camera_name': 'mono_camera',
            'image_width': 1920,
            'image_height': 1080,
            'frame_id': 'camera_frame'
        }]
    )
    nodes.append(camera_info_pub)
    
    # RViz2 node with single camera config
    rviz_config_file = os.path.join(package_dir, 'config', 'monocamera_rviz.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        output='screen'
    )
    nodes.append(rviz_node)
    
    return LaunchDescription(nodes)