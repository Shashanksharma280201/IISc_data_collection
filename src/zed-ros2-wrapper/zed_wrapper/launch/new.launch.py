import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Path to the original zed_camera.launch.py
    zed_launch_path = PathJoinSubstitution([
        get_package_share_directory('zed_wrapper'),
        'launch',
        'zed_camera.launch.py'
    ])

    # First ZED camera (zed1)
    zed1_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(zed_launch_path),
        launch_arguments={
            'camera_name': 'zed1',
            'namespace': 'zed1',
            'camera_model': 'zedx',
            'node_name': 'zed1_node',
        }.items()
    )

    # Second ZED camera (zed2)
    zed2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(zed_launch_path),
        launch_arguments={
            'camera_name': 'zed2',
            'namespace': 'zed2',
            'camera_model': 'zedx',
            'node_name': 'zed2_node',
        }.items()
    )

    # Static TF from lidar_imu_link to zed1
    static_tf_zed1 = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
            '0.1', '0.0', '0.0',     # X Y Z
            '0', '0', '0', '1',       # QX QY QZ QW (identity rotation)
            'lidar_imu_link',        # parent frame
            'zed1/zed2i_base_link'   # child frame (check this frame name from ZED node)
        ],
        output='screen'
    )

    # Static TF from lidar_imu_link to zed2
    static_tf_zed2 = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'tf2_ros', 'static_transform_publisher',
            '-0.1', '0.0', '0.0',     # X Y Z
            '0', '0', '0', '1',       # QX QY QZ QW
            'lidar_imu_link',
            'zed2/zed2i_base_link'
        ],
        output='screen'
    )

    return LaunchDescription([
        zed1_launch,
        zed2_launch,
        static_tf_zed1,
        static_tf_zed2
    ])
