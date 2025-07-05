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
import json
import shutil
import time
import zipfile

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
        self.duration = timedelta(minutes=3)
        self.fps = 30
        self.upload_threads = []

        self.current_session = None
        self.session_start_time = datetime.now()
        self.start_new_session()

        upload_thread = threading.Thread(target=self.upload_loop, daemon=True)
        upload_thread.start()

    def start_new_session(self):
        self.current_session = datetime.now().strftime('session_%Y%m%d_%H%M%S')
        os.makedirs(self.current_session, exist_ok=True)
        for cam in self.camera_topics:
            os.makedirs(f"{self.current_session}/{cam}", exist_ok=True)
            self.create_subscription(Image, self.camera_topics[cam], self.generate_callback(cam), 10)
            self.last_rotation_time[cam] = datetime.now()

        os.makedirs(f"{self.current_session}/bag", exist_ok=True)
        self.start_new_bag()
        self.create_metadata_file()
        self.get_logger().info(f"Started new session: {self.current_session}")

    def rotate_session(self):
        self.stop_current_bag()
        for writer in self.video_writers.values():
            writer.release()
        self.video_writers.clear()
        self.start_new_session()

    def create_metadata_file(self):
        metadata = {
            "session": self.current_session,
            "start_time": datetime.now().isoformat(),
            "camera_topics": self.camera_topics,
            "bag_topics": self.bag_topics,
            "fps": self.fps,
            "duration_minutes": self.duration.total_seconds() / 60
        }
        with open(f"{self.current_session}/metadata.json", "w") as f:
            json.dump(metadata, f, indent=2)

    def generate_callback(self, cam):
        def callback(msg):
            try:
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            except Exception as e:
                self.get_logger().error(f"[{cam}] Error converting image: {e}")
                return

            now = datetime.now()

            if cam not in self.video_writers or (now - self.last_rotation_time[cam]) >= self.duration:
                if cam in self.video_writers:
                    self.video_writers[cam].release()
                    file_to_upload = self.last_file_path[cam]
                    thread = threading.Thread(target=self.upload_file_to_s3, args=(cam, file_to_upload))
                    thread.start()
                    self.upload_threads.append(thread)

                if (now - self.session_start_time) >= self.duration:
                    self.rotate_session()
                    self.session_start_time = now

                filename = now.strftime(f"{self.current_session}/{cam}/%Y%m%d_%H%M%S.mp4")
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
                self.stop_current_bag()
                self.start_new_bag()

        return callback

    def start_new_bag(self):
        now = datetime.now()
        bag_dir = os.path.join(self.current_session, "bag", now.strftime("%Y%m%d_%H%M%S"))
        os.makedirs(bag_dir, exist_ok=True)
        self.current_bag_path = bag_dir

        try:
            self.bag_process = subprocess.Popen(
                ['ros2', 'bag', 'record', '-o', os.path.join(bag_dir, 'data')] + self.bag_topics,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            self.get_logger().info(f"Started new rosbag at: {bag_dir}")
        except Exception as e:
            self.get_logger().error(f"Failed to start rosbag: {e}")

        self.last_bag_time = datetime.now()

    def stop_current_bag(self):
        if hasattr(self, 'bag_process') and self.bag_process:
            self.bag_process.send_signal(signal.SIGINT)
            try:
                self.bag_process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.get_logger().warning("Bag process did not terminate in time, killing it.")
                self.bag_process.kill()
            time.sleep(2)
            self.get_logger().info(f"Stopped rosbag: {self.current_bag_path}")
            self.bag_process = None

    def zip_session(self, session_path):
        zip_path = f"{session_path}.zip"
        with zipfile.ZipFile(zip_path, 'w', zipfile.ZIP_DEFLATED) as zipf:
            for root, _, files in os.walk(session_path):
                for file in files:
                    full_path = os.path.join(root, file)
                    arcname = os.path.relpath(full_path, start=session_path)
                    zipf.write(full_path, arcname)
        return zip_path

    def upload_loop(self):
        while True:
            time.sleep(10)
            sessions = [d for d in os.listdir('.') if d.startswith('session_') and os.path.isdir(d)]
            sessions.sort()
            for session in sessions:
                if session == self.current_session:
                    continue
                zip_path = self.zip_session(session)
                success = self.upload_session_to_s3(zip_path)
                if success:
                    shutil.rmtree(session)
                    os.remove(zip_path)
                    self.get_logger().info(f"Uploaded and deleted session: {session}")
                else:
                    self.get_logger().warning(f"Failed to upload session: {session}")

            if not sessions:
                self.get_logger().info("No unuploaded session found. Exiting...")
                os._exit(0)

    def upload_session_to_s3(self, zip_file_path):
        try:
            self.s3_client.upload_file(zip_file_path, self.s3_bucket, os.path.basename(zip_file_path))
            self.get_logger().info(f"Uploaded {zip_file_path} to S3.")
            return True
        except Exception as e:
            self.get_logger().error(f"Upload failed for {zip_file_path}: {e}")
            return False

    def upload_file_to_s3(self, cam, file_path):
        s3_key = f"{file_path}"
        try:
            self.s3_client.upload_file(file_path, self.s3_bucket, s3_key)
            self.get_logger().info(f"[{cam}] Uploaded {s3_key} to S3.")
            os.remove(file_path)
        except Exception as e:
            self.get_logger().error(f"[{cam}] Upload failed for {s3_key}: {e}")


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
