import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from datetime import datetime, timedelta
import boto3
from botocore.exceptions import BotoCoreError, ClientError
import subprocess
import signal
import threading
import shutil

class MultiCameraRecorder(Node):
    def __init__(self):
        super().__init__('multi_camera_recorder')

        self.get_logger().info("Initializing MultiCameraRecorder...")

        self.s3_bucket = "iisc-data-collection"
        self.s3_client = boto3.client('s3')

        self.bridge = CvBridge()
        self.camera_topics = {
            'cam0': '/cam0/image_raw',
            'cam1': '/cam1/image_raw',
            'cam2': '/cam2/image_raw',
            'zed1_left': '/zed1/zed1/left/image_rect_color',
            'zed1_right': '/zed1/zed1/right/image_rect_color',
            'zed2_left': '/zed2/zed2/left/image_rect_color',
            'zed2_right': '/zed2/zed2/right/image_rect_color',
        }

        self.bag_topics = [
            '/rslidar_imu_data',
            '/rslidar_points',
            '/zed1/zed1/imu/data',
            '/zed2/zed2/imu/data',
        ]

        self.video_writers = {}
        self.last_rotation_time = {}
        self.last_file_path = {}
        self.duration = timedelta(minutes=1)
        self.fps = 30
        self.upload_threads = []

        self.session_dir = ""
        self.video_dirs = {}
        self.bag_dir = ""

        for cam in self.camera_topics:
            self.create_subscription(Image, self.camera_topics[cam], self.generate_callback(cam), 10)

        self.bag_process = None
        self.last_bag_time = datetime.now()
        self.current_bag_path = ""
        self.create_new_session_folder()
        self.start_new_bag()

        self.get_logger().info("Session setup complete. Uploading previous sessions if any...")
        self.upload_pending_sessions()

    def create_new_session_folder(self):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.session_dir = f"session_{timestamp}"
        os.makedirs(self.session_dir, exist_ok=True)
        for cam in self.camera_topics:
            cam_dir = os.path.join(self.session_dir, cam)
            os.makedirs(cam_dir, exist_ok=True)
            self.video_dirs[cam] = cam_dir
        self.bag_dir = os.path.join(self.session_dir, "bag")
        os.makedirs(self.bag_dir, exist_ok=True)

    def generate_callback(self, cam):
        def callback(msg):
            try:
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            except Exception as e:
                self.get_logger().error(f"[{cam}] Error converting image: {e}")
                return

            now = datetime.now()

            if cam not in self.video_writers:
                filename = now.strftime(f"{self.video_dirs[cam]}/%Y%m%d_%H%M%S.mp4")
                height, width, _ = frame.shape
                writer = cv2.VideoWriter(filename, cv2.VideoWriter_fourcc(*'mp4v'), self.fps, (width, height))
                if not writer.isOpened():
                    self.get_logger().error(f"[{cam}] Failed to open video writer for {filename}")
                    return
                self.video_writers[cam] = writer
                self.last_rotation_time[cam] = now
                self.last_file_path[cam] = filename
                self.get_logger().info(f"[{cam}] Started new recording: {filename}")

            self.video_writers[cam].write(frame)

            if (now - self.last_bag_time) >= self.duration:
                self.get_logger().debug("Time to rotate session.")
                self.stop_current_bag()
                for writer in self.video_writers.values():
                    writer.release()
                self.upload_session_to_s3(self.session_dir, os.path.basename(self.session_dir))
                self.video_writers.clear()
                self.create_new_session_folder()
                self.start_new_bag()

        return callback

    def start_new_bag(self):
        now = datetime.now()
        bag_folder_name = now.strftime("%Y%m%d_%H%M%S")
        self.current_bag_path = os.path.join(self.bag_dir, bag_folder_name)
        os.makedirs(self.bag_dir, exist_ok=True)  # Ensure bag dir exists
        try:
            # Use only the folder name for `-o`, and `cwd` to set working directory
            self.bag_process = subprocess.Popen(
                ['ros2', 'bag', 'record', '-o', bag_folder_name] + self.bag_topics,
                cwd=self.bag_dir,  # run inside the bag dir
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            self.get_logger().info(f"Started new rosbag at: {self.current_bag_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to start rosbag: {e}")
        self.last_bag_time = now


    def stop_current_bag(self):
        if self.bag_process:
            self.bag_process.send_signal(signal.SIGINT)
            self.bag_process.wait()
            self.get_logger().info(f"Stopped rosbag: {self.current_bag_path}")
            self.bag_process = None

    def upload_pending_sessions(self):
        for folder in os.listdir('.'):
            if folder.startswith('session_'):
                session_path = os.path.abspath(folder)
                s3_key = folder
                marker_file = os.path.join(session_path, '.uploaded')
                if os.path.exists(marker_file):
                    continue
                thread = threading.Thread(target=self.upload_session_to_s3, args=(session_path, s3_key))
                thread.start()
                self.upload_threads.append(thread)

    def upload_session_to_s3(self, session_path, s3_key):
        try:
            for root, _, files in os.walk(session_path):
                for file in files:
                    if file == '.uploaded':
                        continue
                    full_path = os.path.join(root, file)
                    rel_path = os.path.relpath(full_path, session_path)
                    self.s3_client.upload_file(full_path, self.s3_bucket, f"{s3_key}/{rel_path}")
                    self.get_logger().info(f"Uploaded {rel_path} to S3 under {s3_key}/")
            with open(os.path.join(session_path, '.uploaded'), 'w') as f:
                f.write('Uploaded to S3')
        except Exception as e:
            self.get_logger().error(f"Failed to upload session {s3_key} to S3: {e}")

    def destroy_node(self):
        self.get_logger().info("Shutting down MultiCameraRecorder...")
        for cam, writer in self.video_writers.items():
            writer.release()
        self.stop_current_bag()
        for t in self.upload_threads:
            t.join()
        self.upload_session_to_s3(self.session_dir, os.path.basename(self.session_dir))
        self.get_logger().info("All uploads completed.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MultiCameraRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("KeyboardInterrupt received. Shutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
