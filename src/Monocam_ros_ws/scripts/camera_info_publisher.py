#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
import numpy as np

class CameraInfoPublisher(Node):
    def __init__(self):
        super().__init__('camera_info_publisher')
        
        # Get parameters
        self.camera_name = self.declare_parameter('camera_name', 'mono_camera').value
        self.image_width = self.declare_parameter('image_width', 1920).value
        self.image_height = self.declare_parameter('image_height', 1080).value
        self.frame_id = self.declare_parameter('frame_id', 'camera_link').value
        
        # Create publisher
        self.camera_info_pub = self.create_publisher(
            CameraInfo, 
            f'{self.camera_name}/camera_info', 
            10
        )
        
        # Create timer to publish camera info
        self.timer = self.create_timer(0.1, self.publish_camera_info)  # 10 Hz
        
        self.get_logger().info(f'Camera info publisher started for {self.camera_name}')
    
    def publish_camera_info(self):
        camera_info = CameraInfo()
        
        # Header
        camera_info.header.stamp = self.get_clock().now().to_msg()
        camera_info.header.frame_id = self.frame_id
        
        # Image dimensions
        camera_info.width = self.image_width
        camera_info.height = self.image_height
        
        # Camera matrix (K) - simplified for demo
        # These values should be calibrated for your specific camera
        fx = fy = 1000.0  # Focal length
        cx = self.image_width / 2.0  # Principal point x
        cy = self.image_height / 2.0  # Principal point y
        
        camera_info.k = [
            fx, 0.0, cx,
            0.0, fy, cy,
            0.0, 0.0, 1.0
        ]
        
        # Distortion coefficients (D) - assuming no distortion for demo
        camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        
        # Rectification matrix (R) - identity for monocular
        camera_info.r = [
            1.0, 0.0, 0.0,
            0.0, 1.0, 0.0,
            0.0, 0.0, 1.0
        ]
        
        # Projection matrix (P)
        camera_info.p = [
            fx, 0.0, cx, 0.0,
            0.0, fy, cy, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]
        
        # Distortion model
        camera_info.distortion_model = 'plumb_bob'
        
        # Publish
        self.camera_info_pub.publish(camera_info)

def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()