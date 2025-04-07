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
        "framerate": 25.0,  # Max FPS at highest resolution
        "frame_id": "cam0",
        "image_width": 3840,  # Max resolution width
        "image_height": 2160,  # Max resolution height
        "video_device": "/dev/video6",
        "pixel_format": "mjpeg2rgb",  # Convert MJPEG to RGB
        # "av_device_format": "MJPEG",
        "io_method": "mmap",
        "brightness": 0,
        "contrast": 48
    }


    config["cam1"] = {
        "camera_name": "cam1",
        "framerate": 10.0,
        "frame_id": "cam1",
        "image_width": 3840,  # Max resolution width
        "image_height": 2160,  # Max resolution height
        "video_device": "/dev/video8",
        "pixel_format": "mjpeg2rgb",  # Convert MJPEG to RGB
        # "av_device_format": "MJPEG",
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




    # # Oakd camera
    # def safe_load_params(config_file, key):
    #     params = {}
    #     if not os.path.exists(config_file):
    #         return params
    #     with open(config_file, 'r') as f:
    #         config = yaml.safe_load(f)
    #         if key in config:
    #             params = config[key]
    #     return params

    # config_file = os.path.join(get_package_share_directory('flo_driver'), 'config.yaml')
    # oakd_params = safe_load_params(config_file, 'oakd')

    # oakd = Node(
    #     package='flo_driver',
    #     executable='oakd.py',
    #     name="oakd",
    #     parameters=[oakd_params] if oakd_params else [],
    # )
    # nodes.append(oakd)





    # Lidar
    # share_dir = get_package_share_directory('ydlidar_ros2_driver')
    # rviz_config_file = os.path.join(share_dir, 'config', 'ydlidar.rviz')
    # parameter_file = LaunchConfiguration('params_file')

    # params_declare = DeclareLaunchArgument('params_file',
    #                                        default_value=os.path.join(
    #                                            share_dir, 'params', 'ydlidar.yaml'),
    #                                        description='Path to the ROS2 parameters file to use.')

    # driver_node = LifecycleNode(
    #     package='ydlidar_ros2_driver',
    #     executable='ydlidar_ros2_driver_node',
    #     name='ydlidar_ros2_driver_node',
    #     output='screen',
    #     emulate_tty=True,
    #     parameters=[parameter_file],
    #     namespace='/'
    # )
    # tf2_node = Node(
    #     package='tf2_ros',
    #     executable='static_transform_publisher',
    #     name='static_tf_pub_laser',
    #     arguments=['0', '0', '0.02', '0', '0', '0', '1', 'base_link', 'laser_frame']
    # )
    # rviz2_node = Node(
    #     package='rviz2',
    #     executable='rviz2',
    #     name='rviz2',
    #     arguments=['-d', rviz_config_file]
    # )

    # launch_actions = [
    #     params_declare,
    #     driver_node,
    #     tf2_node,
    #     rviz2_node
    # ]

    return LaunchDescription(nodes)
