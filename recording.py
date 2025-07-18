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

class MultiCameraRecorder(Node):
    def __init__(self):
        super().__init__('multi_camera_recorder')

        self.get_logger().info("Initializing MultiCameraRecorder...")

        # AWS S3 setup
        self.s3_bucket = "iisc-data-collection"
        self.s3_client = boto3.client('s3')

        # Camera topics
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

        # ROS bag topics
        self.bag_topics = [
            '/rslidar_imu_data',
            '/rslidar_points',
            '/zed1/zed1/imu/data',
            '/zed2/zed2/imu/data',
        ]

        self.video_writers = {}
        self.last_rotation_time = {}
        self.last_file_path = {}
        self.duration = timedelta(minutes=5)
        self.fps = 30

        self.upload_threads = []

        os.makedirs("videos", exist_ok=True)
        os.makedirs("failed", exist_ok=True)
        os.makedirs("bags", exist_ok=True)

        for cam, topic in self.camera_topics.items():
            self.create_subscription(Image, topic, self.generate_callback(cam), 10)
            os.makedirs(f"videos/{cam}", exist_ok=True)
            os.makedirs(f"failed/{cam}", exist_ok=True)
            self.last_rotation_time[cam] = datetime.now()

        self.bag_process = None
        self.last_bag_time = datetime.now()
        self.current_bag_path = ""
        self.start_new_bag()

        self.get_logger().info("Checking for old video and bag files to upload...")
        self.upload_pending_files()

        self.get_logger().info("MultiCameraRecorder initialized.")

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
                    thread = threading.Thread(target=self.upload_to_s3, args=(cam, file_to_upload))
                    thread.start()
                    self.upload_threads.append(thread)

                filename = now.strftime(f"videos/{cam}/%Y%m%d_%H%M%S.mp4")
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
                self.get_logger().debug("Time to rotate rosbag.")
                self.stop_current_bag()
                self.start_new_bag()

        return callback

    def start_new_bag(self):
        now = datetime.now()
        bag_dir = now.strftime("bags/%Y%m%d_%H%M%S")
        self.current_bag_path = bag_dir

        try:
            self.bag_process = subprocess.Popen(
                ['ros2', 'bag', 'record', '-o', bag_dir] + self.bag_topics,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            self.get_logger().info(f"Started new rosbag at: {bag_dir}")
        except Exception as e:
            self.get_logger().error(f"Failed to start rosbag: {e}")

        self.last_bag_time = datetime.now()

    def stop_current_bag(self):
        if self.bag_process:
            self.bag_process.send_signal(signal.SIGINT)
            self.bag_process.wait()
            self.get_logger().info(f"Stopped rosbag: {self.current_bag_path}")

            thread = threading.Thread(target=self.upload_bag_to_s3, args=(self.current_bag_path,))
            thread.start()
            self.upload_threads.append(thread)

            self.bag_process = None

    def upload_to_s3(self, cam, file_path):
        s3_key = os.path.basename(file_path)
        try:
            self.get_logger().debug(f"[{cam}] Uploading {s3_key} to S3...")
            self.s3_client.upload_file(file_path, self.s3_bucket, f"{cam}/{s3_key}")
            self.get_logger().info(f"[{cam}] Uploaded {s3_key} to S3.")
            os.remove(file_path)
            self.get_logger().debug(f"[{cam}] Deleted local file after upload: {file_path}")
        except (BotoCoreError, ClientError) as e:
            self.get_logger().error(f"[{cam}] Failed to upload {s3_key}: {e}")
            failed_path = f"failed/{cam}/{s3_key}"
            os.rename(file_path, failed_path)
            self.get_logger().warning(f"[{cam}] Moved to failed folder: {failed_path}")


    def upload_bag_to_s3(self, bag_path):
        try:
            self.get_logger().debug(f"Uploading bag folder {bag_path} to S3...")
            for file in os.listdir(bag_path):
                if file.endswith(".db3"):
                    full_path = os.path.join(bag_path, file)
                    s3_key = f"bags/{os.path.basename(bag_path)}/{file}"
                    self.s3_client.upload_file(full_path, self.s3_bucket, s3_key)
                    self.get_logger().info(f"Uploaded {s3_key} to S3.")
                    os.remove(full_path)
                    self.get_logger().debug(f"Deleted local file after upload: {full_path}")
            # Optionally delete the empty directory
            os.rmdir(bag_path)
            self.get_logger().debug(f"Deleted bag directory: {bag_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to upload rosbag to S3: {e}")


    def upload_pending_files(self):
        for cam in self.camera_topics.keys():
            video_dir = f"videos/{cam}"
            if os.path.exists(video_dir):
                for file in os.listdir(video_dir):
                    if file.endswith(".mp4"):
                        file_path = os.path.join(video_dir, file)
                        thread = threading.Thread(target=self.upload_to_s3, args=(cam, file_path))
                        thread.start()
                        self.upload_threads.append(thread)

        if os.path.exists("bags"):
            for bag_dir in os.listdir("bags"):
                full_dir_path = os.path.join("bags", bag_dir)
                if os.path.isdir(full_dir_path):
                    thread = threading.Thread(target=self.upload_bag_to_s3, args=(full_dir_path,))
                    thread.start()
                    self.upload_threads.append(thread)

    def destroy_node(self):
        self.get_logger().info("Shutting down MultiCameraRecorder...")

        for cam, writer in self.video_writers.items():
            writer.release()
            if cam in self.last_file_path:
                thread = threading.Thread(target=self.upload_to_s3, args=(cam, self.last_file_path[cam]))
                thread.start()
                self.upload_threads.append(thread)

        self.stop_current_bag()

        for t in self.upload_threads:
            t.join()

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
