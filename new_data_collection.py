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

def log_rosbag_output(stream):
        for line in iter(stream.readline, b''):
            print(f"[rosbag2] {line.decode().strip()}")


class MultiCameraRecorder(Node):
    def __init__(self):
        super().__init__('multi_camera_recorder')

        # Root path setup
        self.root_dir = os.path.dirname(os.path.abspath(__file__))
        self.videos_dir = os.path.join(self.root_dir, "videos")
        self.failed_dir = os.path.join(self.root_dir, "failed")
        self.bags_dir = os.path.join(self.root_dir, "bags")

        # Create necessary directories
        os.makedirs(self.videos_dir, exist_ok=True)
        os.makedirs(self.failed_dir, exist_ok=True)
        os.makedirs(self.bags_dir, exist_ok=True)

        # AWS S3 setup
        self.s3_bucket = "iisc-data-store"
        self.s3_client = boto3.client('s3')

        # Topic and camera setup
        self.bridge = CvBridge()
        self.camera_topics = {
            'cam0': '/cam0/image_raw',
            'cam1': '/cam1/image_raw',
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

        for cam, topic in self.camera_topics.items():
            self.create_subscription(Image, topic, self.generate_callback(cam), 10)

            cam_video_dir = os.path.join(self.videos_dir, cam)
            cam_failed_dir = os.path.join(self.failed_dir, cam)

            os.makedirs(cam_video_dir, exist_ok=True)
            os.makedirs(cam_failed_dir, exist_ok=True)

            self.last_rotation_time[cam] = datetime.now()

        self.bag_process = None
        self.last_bag_time = datetime.now()
        self.current_bag_path = ""
        self.start_new_bag()

    def generate_callback(self, cam):
        def callback(msg):
            try:
                frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            except Exception as e:
                self.get_logger().error(f"Error converting image for {cam}: {e}")
                return

            now = datetime.now()
            if cam not in self.video_writers or (now - self.last_rotation_time[cam]) >= self.duration:
                if cam in self.video_writers:
                    self.video_writers[cam].release()
                    file_to_upload = self.last_file_path[cam]
                    self.upload_to_s3(cam, file_to_upload)

                filename = now.strftime(os.path.join(self.videos_dir, cam, "%Y%m%d_%H%M%S.mp4"))
                height, width, _ = frame.shape
                writer = cv2.VideoWriter(
                    filename,
                    cv2.VideoWriter_fourcc(*'mp4v'),
                    self.fps,
                    (width, height)
                )
                if not writer.isOpened():
                    self.get_logger().error(f"{cam}: Failed to open video writer for {filename}")
                    return

                self.video_writers[cam] = writer
                self.last_rotation_time[cam] = now
                self.last_file_path[cam] = filename
                self.get_logger().info(f"{cam}: Started new recording {filename}")

            self.video_writers[cam].write(frame)

            # Handle rosbag rotation
            if (now - self.last_bag_time) >= self.duration:
                self.stop_current_bag()
                self.start_new_bag()
        return callback


    


    def start_new_bag(self):
        now = datetime.now()
        bag_dir = now.strftime(os.path.join(self.bags_dir, "%Y%m%d_%H%M%S"))
        os.makedirs(bag_dir, exist_ok=True)
        self.current_bag_path = bag_dir

        try:
            self.bag_process = subprocess.Popen(
                ['ros2', 'bag', 'record', '-o', bag_dir] + self.bag_topics,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT
            )
            # Log output in a background thread
            threading.Thread(target=log_rosbag_output, args=(self.bag_process.stdout,), daemon=True).start()

            self.get_logger().info(f"Started new rosbag at {bag_dir}")
        except Exception as e:
            self.get_logger().error(f"Failed to start rosbag: {e}")

        self.last_bag_time = datetime.now()



    def stop_current_bag(self):
        if self.bag_process:
            self.bag_process.send_signal(signal.SIGINT)
            self.bag_process.wait()
            self.get_logger().info(f"Stopped rosbag: {self.current_bag_path}")
            self.upload_bag_to_s3(self.current_bag_path)
            self.bag_process = None

    def upload_to_s3(self, cam, file_path):
        s3_key = os.path.basename(file_path)
        try:
            self.s3_client.upload_file(file_path, self.s3_bucket, f"{cam}/{s3_key}")
            self.get_logger().info(f"{cam}: Uploaded {s3_key} to S3.")
        except (BotoCoreError, ClientError) as e:
            self.get_logger().error(f"{cam}: Failed to upload {s3_key}. Error: {e}")
            failed_path = os.path.join(self.failed_dir, cam, s3_key)
            os.rename(file_path, failed_path)
            self.get_logger().info(f"{cam}: Moved to failed folder: {failed_path}")

    def upload_bag_to_s3(self, bag_path):
        try:
            for file in os.listdir(bag_path):
                if file.endswith(".db3"):
                    full_path = os.path.join(bag_path, file)
                    s3_key = f"bags/{os.path.basename(bag_path)}/{file}"
                    self.s3_client.upload_file(full_path, self.s3_bucket, s3_key)
                    self.get_logger().info(f"Uploaded {s3_key} to S3.")
        except Exception as e:
            self.get_logger().error(f"Failed to upload rosbag to S3: {e}")

    def destroy_node(self):
        for cam, writer in self.video_writers.items():
            writer.release()
            if cam in self.last_file_path:
                self.upload_to_s3(cam, self.last_file_path[cam])
        self.stop_current_bag()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = MultiCameraRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
