#!/usr/bin/env python3

import os
import math
from threading import Lock, Thread

import rclpy
from rclpy.node import Node
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from rclpy.executors import MultiThreadedExecutor

from flo_msgs.srv import StartRecordingPath, StopRecordingPath, GetRobotPose
from flo_msgs.msg import Path2D, Point2D, Pose2D
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

from flo_utils.ros2_utils import spin, Rate, lookup_tf
from flo_perception_control.utils import yaw_from_quaternion


class RecordPath(Node):
    def __init__(self):
        super().__init__('record_path')

        self.declare_parameter('path_to_ws', 'flo_nav_ws')
        self.path_to_paths_dir = os.path.join(
            os.path.expanduser('~'),
            self.get_parameter(
                'path_to_ws'
            ).get_parameter_value().string_value,
            "paths"
        )

        self.declare_parameter('debug', False)
        self.debug = self.get_parameter(
            'debug'
        ).get_parameter_value().bool_value

        self.declare_parameter('path_resolution', 0.1)
        self.path_resolution = self.get_parameter(
            'path_resolution').get_parameter_value().double_value

        self.recording = False
        self.recording_lock = Lock()
        self.record_thread = None
        self.recorded_path = []
        self.recording_failed = False
        self.recording_failed_msg = ""
        self.record_frame = ""
        self.record_name = ""
        self.start_recording_path_service = self.create_service(
            StartRecordingPath,
            'start_recording_path',
            self.start_recording_cb
        )

        self.stop_recording_path_service = self.create_service(
            StopRecordingPath,
            'stop_recording_path',
            self.stop_recording_cb
        )

        self.get_robot_pose_service = self.create_service(
            GetRobotPose,
            "get_robot_pose",
            self.get_robot_pose_cb
        )

        self.path_pub = self.create_publisher(
            Path,
            "recorded_path",
            10
        )

        self.is_localized = False
        self.create_all_subscriptions()

    def create_all_subscriptions(self):
        self.localization_state_sub = self.create_subscription(
            Bool,
            "/localization_status",
            self.localization_status_cb,
            10
        )
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

    def destroy_all_subscriptions(self):
        self.destroy_subscription(self.localization_state_sub)
        self._tf_listener.unregister()
        self._tf_listener = None

    def localization_status_cb(self, msg):
        self.is_localized = msg.data

    def start_recording_cb(self, req, res):
        if not self.recording:
            self.recording_lock.acquire()
            self.recording = True
            self.record_frame = req.frame
            self.record_name = req.name
            self.record_thread = Thread(target=self.record_thread_cb)
            self.record_thread.start()
            res.success = True
            self.recording_lock.release()
        else:
            res.success = False
            res.message = "Already recording a path!"
        return res

    def stop_recording_cb(self, req, res):
        if self.recording:
            self.recording_lock.acquire()
            # Stop record_thread
            self.recording = False
            self.record_thread.join()

            if self.recording_failed:
                res.success = False
                res.message = self.recording_failed_msg
            else:
                path2d = Path2D()
                res.success = True
                for point in self.recorded_path:
                    path2d.points.append(
                        Point2D(x=point[0], y=point[1])
                    )
                res.path = path2d

                # Save path to file
                if self.record_name != "":
                    if not os.path.exists(self.path_to_paths_dir):
                        os.mkdir(self.path_to_paths_dir)
                    fd = open(
                        os.path.join(self.path_to_paths_dir, f"{self.record_name}.csv"),
                        "w"
                    )
                    fd.write("index,x,y\n")
                    for index, point in enumerate(self.recorded_path):
                        fd.write(f"{index},{point[0]},{point[1]}\n")
                    fd.close()

            self.recorded_path = []
            self.record_frame = ""
            self.record_Name = ""
            self.recording_failed = False
            self.recording_failed_msg = ""
            self.recording_lock.release()
        else:
            res.success = False
            res.message = "No path is being recorded currently!"
        return res
    
    def get_robot_pose_cb(self, req, res):
        frame = req.frame
        T_bf_frame = lookup_tf(self._tf_buffer, 
                               frame, 
                               "base_footprint", 
                               max_retry=20,
                               logger=self.get_logger())
        if T_bf_frame is None:
            res.success = False
            res.message = "Failed to get robot pose"
            return res
        
        pose = Pose2D()
        pose.point.x = T_bf_frame.transform.translation.x
        pose.point.y = T_bf_frame.transform.translation.y
        pose.yaw = yaw_from_quaternion(
            T_bf_frame.transform.rotation.w,
            T_bf_frame.transform.rotation.z
        )
        res.success = True
        res.message = ""
        res.pose = pose
        return res

    def record_thread_cb(self):
        rate = Rate(10)
        while rclpy.ok() and self.recording:
            tf = lookup_tf(self._tf_buffer, 
                           self.record_frame, 
                           "base_footprint", 
                           max_retry=20,
                           logger=self.get_logger())
            if tf is None:
                self.recording_failed = True
                self.recording_failed_msg = f"Failed to get transform of {self.record_frame}"
                return

            if self.record_frame != "odom" and self.is_localized is False:
                self.recording_failed = True
                self.recording_failed_msg = "Failed to record path as not localized!"
                return

            if len(self.recorded_path) == 0:
                self.recorded_path.append([
                    tf.transform.translation.x,
                    tf.transform.translation.y
                ])
                continue

            dx = math.sqrt(
                (tf.transform.translation.x - self.recorded_path[-1][0])**2 +
                (tf.transform.translation.y - self.recorded_path[-1][1])**2
            )

            if dx >= self.path_resolution:
                self.recorded_path.append([
                    tf.transform.translation.x,
                    tf.transform.translation.y
                ])

            if self.debug:
                self.publish_recorded_path()
            rate.sleep()

    def publish_recorded_path(self):
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = self.record_frame
        poses = []
        for point in self.recorded_path:
            pose = PoseStamped()
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            poses.append(pose)
        path.poses = poses
        self.path_pub.publish(path)


def main(args=None):
    rclpy.init(args=args)
    record_path = RecordPath()
    multi_thread_executor = MultiThreadedExecutor()
    multi_thread_executor.add_node(record_path)
    try:
        spin(record_path, executor=multi_thread_executor, hz=125)
    except KeyboardInterrupt:
        if record_path.recording:
            record_path.recording = False
            record_path.record_thread.join()
        record_path.destroy_node()
        print("Killing record_path!")


if __name__ == "__main__":
    main()
