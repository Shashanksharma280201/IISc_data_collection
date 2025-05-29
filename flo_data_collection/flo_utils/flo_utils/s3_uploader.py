#!/usr/bin/env python3

import os
import sys
import shutil
import multiprocessing
from threading import Thread, Lock

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

import boto3
import yaml
from botocore.exceptions import ClientError
from flo_msgs.msg import S3UploadStatus
from flo_msgs.srv import EnableUploading, UpdateUploadParams

from flo_utils.ros2_utils import spin, Rate
from boto3.s3.transfer import TransferConfig

def format_size(size_in_bytes):
    """
    Convert the size from bytes to other units like KB, MB, GB, etc., 
    and format it to a human-readable form.
    """
    # Define the suffixes for each unit
    for unit in ['bytes', 'KB', 'MB', 'GB', 'TB', 'PB', 'EB', 'ZB', 'YB']:
        if size_in_bytes < 1024.0:
            return f"{size_in_bytes:3.3f} {unit}"
        size_in_bytes /= 1024.0
    return f"{size_in_bytes:.3f} YB"  # Handle extremely large values


class UploadState:
    def __init__(self, sessions_to_upload):
        self.uploading = False

        self.sessions_remaining = sessions_to_upload
        self.current_session = ""
        self.current_session_size = 0.0
        self.current_session_files = 0
        self.current_session_files_done = 0

        self.current_file = ""
        self.current_file_size = 0.0

    def set_uploading_status(self, uploading):
        self.uploading = uploading

    def set_current_session(self, name, num_files, size, files_done):
        self.current_session = name
        self.current_session_size = size

        self.current_session_files = num_files
        self.current_session_files_done = files_done

    def file_upload_done(self):
        self.current_session_files_done += 1

    def set_current_file(self, name, size):
        self.current_file = name
        self.current_file_size = size

    def session_done(self):
        self.sessions_remaining -= 1

    def to_msg(self):
        msg = S3UploadStatus()
        msg.is_uploading = self.uploading
        msg.sessions_remaining = self.sessions_remaining
        
        msg.current_session = self.current_session
        msg.current_session_size = format_size(self.current_session_size)

        msg.current_session_done = self.current_session_files_done
        msg.current_session_total_files = self.current_session_files

        msg.current_file = self.current_file
        msg.current_file_size = format_size(self.current_file_size)

        return msg


class S3Uploader(Node):
    def __init__(self):
        super().__init__('s3_uploader')
        self.declare_parameter('aws_access_key_id', "")
        self.declare_parameter('aws_secret_access_key', "")
        self.declare_parameter('bucket_name', "")
        self.declare_parameter('bucket_region', "")
        self.declare_parameter('data_directory', "")
        self.declare_parameter('identity', "")
        self.declare_parameter('tail_file', 'session_info.yaml')
        self.declare_parameter('max_bandwidth_KBps', 100)
        self.declare_parameter('max_concurrency', 1)
        self.aws_access_key_id = self.get_parameter(
            'aws_access_key_id').get_parameter_value().string_value
        self.aws_secret_access_key = self.get_parameter(
            'aws_secret_access_key').get_parameter_value().string_value
        self.bucket_name = self.get_parameter(
            'bucket_name').get_parameter_value().string_value
        self.bucket_region = self.get_parameter(
            'bucket_region').get_parameter_value().string_value
        self.data_directory = self.get_parameter(
            'data_directory').get_parameter_value().string_value
        self.identity = self.get_parameter(
            'identity').get_parameter_value().string_value
        self.tail_file = self.get_parameter(
            'tail_file').get_parameter_value().string_value
        self.max_bandwidth_KBps = self.get_parameter(
            'max_bandwidth_KBps').get_parameter_value().integer_value
        self.max_concurrency = self.get_parameter(
            'max_concurrency').get_parameter_value().integer_value

        if not os.path.isdir(self.data_directory):
            self.get_logger().fatal("Data directory passed does not exist!")
            sys.exit(1)

        self.sts = boto3.client(
            'sts',
            aws_access_key_id=self.aws_access_key_id,
            aws_secret_access_key=self.aws_secret_access_key,
            region_name=self.bucket_region
        )

        self._is_ready = False
        try:
            self.sts.get_caller_identity()
            self._is_ready = True
        except ClientError as e:
            self.get_logger().error(f"Invalid aws credentials : {e}")
            self._is_ready = False
        except BaseException as e:
            self.get_logger().error(f"Error in validating aws credentials : {e}")
            self._is_ready = False

        self.s3 = boto3.client(
            's3',
            aws_access_key_id=self.aws_access_key_id,
            aws_secret_access_key=self.aws_secret_access_key,
            region_name=self.bucket_region
        )

        self.enable_uploading = False
        self.is_uploading = False
        self.uploading_thread = None
        self.upload_delay_timer = None

        self._current_upload_process = None
        self._status_update_lock = Lock()
        self._current_session_desc = None

        if self._is_ready:
            self.get_logger().info("Ready to upload files.")
        
        self.create_service(
            EnableUploading,
            "/s3_uploader/enable",
            callback=self.s3_upload_enable_cb,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        self.create_service(
            UpdateUploadParams,
            "/s3_uploader/params/update",
            callback=self.s3_uploader_update_params_cb,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.s3_upload_status_pub = self.create_publisher(
            S3UploadStatus,
            "/s3_uploader/status",
            qos
        )

        self._upload_state = None

        self.s3_upload_enable_cb(
            EnableUploading.Request(enable=True, upload_delay=0),
            EnableUploading.Response()
        )
    
    def _publish_current_upload_state(self):
        if self._upload_state is None:
            return
        
        msg = self._upload_state.to_msg()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "s3_uploader"

        if rclpy.ok():
            self.s3_upload_status_pub.publish(msg)
        
    def _upload_thread_cb(self):
        # 0. print params
        self.get_logger().info(f"Uploading {self.data_directory} to s3")
        self.get_logger().info(f"Bucket : {self.bucket_name}," +
                               f"Region : {self.bucket_region}" +
                               f"Folder : {self.identity}")
        
        self._wait_till_ready()

        # 1. get list of sessions in the data dir
        sessions = os.listdir(self.data_directory)
        # 2. sort based on time (ascending)
        sessions = sorted(sessions)

        self._upload_state = UploadState(len(sessions))
        self._upload_state.set_uploading_status(self.enable_uploading)

        self.is_uploading = True
        # 3. select the first one
        for session in sessions:
            if not self.enable_uploading:
                self._upload_state.set_uploading_status(self.enable_uploading)
                self._publish_current_upload_state()
                self.is_uploading = False
                return
            upload_done = False
            while not upload_done:
                if not self.enable_uploading:
                    self._upload_state.set_uploading_status(self.enable_uploading)
                    self._publish_current_upload_state()
                    self.is_uploading = False
                    return
                # 3. upload session
                success = self._upload_session_dir(session)
                # 4. if all files are uploaded, delete folder
                if success:
                    self._upload_state.session_done()
                    self._publish_current_upload_state()

                    shutil.rmtree(os.path.join(self.data_directory, session))
                    self.get_logger().info(
                        f"Uploaded session : {session} to s3 bucket")
                    upload_done = True
                else:
                    if not self.enable_uploading:
                        self.get_logger().warn(f"Aborted uploading of {session}")
                        continue

                    self.get_logger().error("Error in uploading session :"
                                           +f" {session}. Retrying ...")

        self.get_logger().info("All session data has been uploaded.")
        self.enable_uploading = False
        self.is_uploading = False
        self._upload_state.set_uploading_status(self.enable_uploading)
        self._publish_current_upload_state()


    def _wait_till_ready(self):
        rate = Rate(hz=0.5)
        while not self._is_ready:
            if not self.enable_uploading:
                return
            try:
                self.sts.get_caller_identity()
                self._is_ready = True
            except (ClientError, BaseException):
                self._is_ready = False
            rate.sleep()

    def _upload_session_dir(self, session):
        session_path = os.path.join(self.data_directory, session)
        if not os.path.isdir(session_path):
            return

        # 3.1. create a temp file .status.yaml (don't upload this)
        self._current_session_desc, _ = self._get_upload_desc(session)

        self._upload_state.set_current_session(
            name=session,
            num_files=self._current_session_desc['status']['total'],
            size=self._current_session_desc['size'],
            files_done=self._current_session_desc['status']['done']
        )

        for file in self._current_session_desc['files']:
            if not self.enable_uploading:
                return False
            file_name = file['name']
            file_path = file['path']
            source_file_path = os.path.join(self.data_directory, file_path)
            target_file_path = f"{self.identity}/{session}/{file_name}"

            self._upload_state.set_current_file(
                file['name'],
                float(file['size'])
            )
            self._publish_current_upload_state()
            # 3.2. for each file: start upload
            self._current_upload_process = multiprocessing.Process(
                target=self._upload_file, args=(
                    source_file_path, target_file_path,)
            )
            self._current_upload_process.start()
            self._current_upload_process.join()
            if not self.enable_uploading:
                return False
            if not self._current_upload_process.exitcode > 0:
                self._upload_state.file_upload_done()
                self._publish_current_upload_state()

                self.get_logger().debug(
                    f"Successfully uploaded {source_file_path} to s3")
                # 3.3. on finish upload, update .status.yaml
                self._update_desc_on_file_upload_success(session, file)
            else:
                return False
            self._current_upload_process = None
        return True

    def _get_upload_desc(self, session):
        session_path = os.path.join(self.data_directory, session)
        if not os.path.isdir(session_path):
            return

        desc_path = os.path.join(session_path, ".status.yaml")
        if os.path.isfile(desc_path):
            desc = {}
            with open(desc_path) as f:
                try:
                    desc = yaml.safe_load(f)
                    if desc is not None:
                        return desc, desc_path
                except yaml.YAMLError:
                    pass

        desc = {}
        desc['size'] = 0
        desc['status'] = {
            'total': 0,
            'done': 0
        }
        desc['files'] = []
        tail_file_path = ''
        for root, _, files in os.walk(session_path):
            for file in files:
                if file.lower() == self.tail_file:
                    tail_file_path = os.path.join(root, file)
                    continue
                if file.lower() == ".status.yaml":
                    continue
                if os.path.islink(os.path.join(root, file)):
                    continue

                head, prefix = os.path.split(root)
                head = head.replace(session_path, "").lstrip(os.sep)
                prefix = os.path.join(head, prefix)

                if prefix == session:
                    prefix = ""
                    
                if prefix == session_path.replace("/", "", 1):
                    prefix = ""
                    
                full_path = os.path.join(root, file)
                file_size = os.path.getsize(full_path)
                desc['files'].append({
                    'name': os.path.join(prefix, file),
                    'path': full_path,
                    'size': file_size
                })
                desc['size'] += file_size
        
        # 3.1.1. create a snapshot of what to upload
        #    ensure last file to be uploaded is session_info.yaml
        desc['files'].append({
            'name': self.tail_file,
            'path': tail_file_path,
            'size': os.path.getsize(tail_file_path)
        })
        desc['size'] += os.path.getsize(tail_file_path)

        desc['status']['total'] = len(desc['files'])
        with open(desc_path, "w") as desc_file:
            yaml.dump(desc, desc_file)
        return desc, desc_path

    def _update_desc_on_file_upload_success(self, session, file):
        session_path = os.path.join(self.data_directory, session)
        if not os.path.isdir(session_path):
            return

        desc_path = os.path.join(session_path, ".status.yaml")
        with open(desc_path) as f:
            desc = yaml.safe_load(f)
        if desc is None:
            return

        # 3.3.1 remove the file that has been uploaded
        # and increment the counter
        desc['status']['done'] += 1
        desc['files'].remove(file)
        self._status_update_lock.acquire()
        with open(desc_path, 'w') as f:
            yaml.dump(desc, f)
        self._status_update_lock.release()

    def _upload_file(self, source_file_path, target_file_path):
        try:
            source_file_path, self.bucket_name, target_file_path
            config = TransferConfig(
                max_concurrency=self.max_concurrency, 
                max_bandwidth=self.max_bandwidth_KBps*1024
            )
            self.s3.upload_file(
                source_file_path, 
                self.bucket_name,
                target_file_path,
                Config=config
            )
        except KeyboardInterrupt:
            return
        except BaseException as e:
            self.get_logger().warn(f"Error in uploading {source_file_path} to s3 : {e}")
            sys.exit(1)

    def _abort_file_upload(self):
        self.get_logger().warn("Aborting file upload ...")
        if self._current_upload_process is None:
            self.get_logger().info("No process to send SIGINT")
            return
        self.get_logger().debug("Killing process ...")
        self._current_upload_process.kill()
        # os.kill(self._current_upload_process.pid, signal.SIGINT)
        self.get_logger().info("Aborted file uploading")

    def _start_uploading_thread(self):
        if self.upload_delay_timer:
            self.upload_delay_timer.cancel()
            self.upload_delay_timer = None
        self.uploading_thread = Thread(
            name="s3-uploader-thread",
            target=self._upload_thread_cb
        )
        self.uploading_thread.start()

    def s3_upload_enable_cb(self, req, res):
        if req.enable:
            if self.enable_uploading:
                if self.is_uploading:
                    res.success = False
                    res.message = "Already uploading."
                    return res
                res.success = False
                res.message = "Already enabled uploading."
                return res

            if self.upload_delay_timer:
                self.upload_delay_timer.cancel()
                self.upload_delay_timer = None

            if req.upload_delay > 0:
                self.upload_delay_timer = self.create_timer(
                    req.upload_delay,
                    callback=self._start_uploading_thread,
                    callback_group=MutuallyExclusiveCallbackGroup()
                )
                self.enable_uploading = True
                res.success = True
                res.message = f"Uploading data to s3 after {req.upload_delay}s"
                return res
            
            self.enable_uploading = True
            self._start_uploading_thread()
            res.success = True
            res.message = "Started uploading data to s3"
            return res
        else:
            if self.enable_uploading:
                if not self.is_uploading:
                    self.enable_uploading = False
                    if self.upload_delay_timer:
                        self.upload_delay_timer.cancel()
                        self.upload_delay_timer = None
                    
                    res.success = True
                    res.message = "Cancelled scheduled upload"
                    return res

                self.enable_uploading = False
                self._abort_file_upload()
                # self.uploading_thread.join()
                # self.uploading_thread = None

                res.success = True
                res.message = "Aborted file uploading"
                return res
            
            res.success = False
            res.message = "Nothing to stop."
            return res
    
    def s3_uploader_update_params_cb(self, req, res):
        self.max_bandwidth_KBps = req.max_bandwidth
        self.max_concurrency = req.max_concurrency

        res.success = True
        return res


def main(args=None):
    rclpy.init(args=args)
    s3Uploader = S3Uploader()
    executor = MultiThreadedExecutor()
    executor.add_node(s3Uploader)
    try:
        spin(s3Uploader, executor, hz=125)
    except KeyboardInterrupt:
        print("Shutting down s3 uploader ...")
        s3Uploader.s3_upload_enable_cb(
            EnableUploading.Request(enable=False), 
            EnableUploading.Response())
    finally:
        s3Uploader.destroy_node()


if __name__ == '__main__':
    main()
