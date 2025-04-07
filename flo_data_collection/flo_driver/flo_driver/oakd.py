#!/usr/bin/env python3

import os
import time
import threading
from threading import Lock
import cv2
import av
import yaml
from cv_bridge import CvBridge
from fractions import Fraction
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from sensor_msgs.msg import Imu, Image
from std_msgs.msg import Bool
import depthai as dai

import pyvirtualcam
from flo_msgs.msg import OakdUsage
from flo_msgs.srv import StartLogging, StopLogging

from flo_utils.ros2_utils import spin

BYTES_TO_MB = 1024 * 1024 # MiB


class OakdDriver(Node):
    def __init__(self):
        super().__init__('oakd_driver')

        self._pipeline = dai.Pipeline()

        self.cv_bridge = CvBridge()

        self.declare_parameter('enable_imu', True)
        self.declare_parameter('enable_rgb_camera', True)
        self.declare_parameter('enable_depth_camera', True)
        self.declare_parameter('usage_timeout', 1.0)
        self.enable_imu = self.get_parameter(
            'enable_imu').get_parameter_value().bool_value
        self.enable_rgb_camera = self.get_parameter(
            'enable_rgb_camera').get_parameter_value().bool_value
        self.enable_depth_camera = self.get_parameter(
            'enable_depth_camera').get_parameter_value().bool_value

        self.declare_parameter('enable_depth_camera_logging', False)
        self.enable_depth_camera_logging = self.get_parameter(
            'enable_depth_camera_logging').get_parameter_value().bool_value
        self.usage_timeout = self.get_parameter(
            'usage_timeout').get_parameter_value().double_value

        self.init_usage()

        if self.enable_imu:
            self.init_imu()

        if self.enable_rgb_camera:
            self.init_rgb_camera()

        if self.enable_depth_camera:
            self.init_depth_camera()

        # logging
        self.logging = False
        self.logging_lock = Lock()
        self.start_logging_service = self.create_service(
            StartLogging,
            'oakd/start_logging',
            self.start_logging_cb
        )
        self.stop_logging_service = self.create_service(
            StopLogging,
            'oakd/stop_logging',
            self.stop_logging_cb
        )
        self.is_oakd_connected_pub = self.create_publisher(Bool, 'oakd/status', 10)
        # observer
        self.usage_time = None
        self.device = None
        self.start_time = self.get_clock().now()
        self.observer_timer = self.create_timer(5.0, self.observer_cb)
        self.oakd_connect_timer = self.create_timer(
                                    0.2,
                                    self.oakd_connection_check,
                                    callback_group = MutuallyExclusiveCallbackGroup())
        
        self.is_oakd_connected = None

    def oakd_connection_check(self):
        if self.is_oakd_connected is None:
            self.is_oakd_connected = True

        msg = Bool()
        msg.data = self.is_oakd_connected
        self.is_oakd_connected_pub.publish(msg)

    def init_usage(self):
        self.declare_parameter('usage_hz', 1.0)
        self.usage_hz = self.get_parameter(
            'usage_hz').get_parameter_value().double_value

        self._oakd_system_logger = self._pipeline.create(dai.node.SystemLogger)
        self._oakd_system_logger.setRate(self.usage_hz)

        self._oakd_usage_xlink_out = self._pipeline.create(dai.node.XLinkOut)
        self._oakd_usage_xlink_out.setStreamName("usage")

        self._oakd_system_logger.out.link(self._oakd_usage_xlink_out.input)

        self.oakd_usage_pub = self.create_publisher(
            OakdUsage,
            "/oakd/usage",
            10
        )

        # logging
        self.logging_usage_fd = None

    def init_imu(self):
        # params
        self.declare_parameter('imu_timeout', 1.0)
        self.declare_parameter('imu_hz', 100)
        self.declare_parameter(
            'imu_acceleration_covariance_diag', [0.0, 0.0, 0.0])
        self.declare_parameter(
            'imu_angular_velocity_covariance_diag', [0.0, 0.0, 0.0])
        self.declare_parameter(
            'imu_orientation_covariance_diag', [0.0, 0.0, 0.0])
        self.imu_timeout = self.get_parameter(
            'imu_timeout').get_parameter_value().double_value
        self.imu_hz = self.get_parameter(
            'imu_hz').get_parameter_value().integer_value
        self.imu_acceleration_covariance_diag = self.get_parameter(
            'imu_acceleration_covariance_diag').get_parameter_value().double_array_value
        self.imu_angular_velocity_covariance_diag = self.get_parameter(
            'imu_angular_velocity_covariance_diag').get_parameter_value().double_array_value
        self.imu_orientation_covariance_diag = self.get_parameter(
            'imu_orientation_covariance_diag').get_parameter_value().double_array_value

        # add imu to pipeline
        self._imu_xlink_out = self._pipeline.create(dai.node.XLinkOut)
        self._imu_xlink_out.setStreamName("imu")

        self._imu = self._pipeline.create(dai.node.IMU)
        self._imu.enableIMUSensor(
            dai.IMUSensor.ACCELEROMETER, self.imu_hz)
        self._imu.enableIMUSensor(
            dai.IMUSensor.GYROSCOPE_CALIBRATED, self.imu_hz)
        self._imu.enableIMUSensor(
            dai.IMUSensor.ROTATION_VECTOR, self.imu_hz)
        self._imu.setBatchReportThreshold(1)
        self._imu.setMaxBatchReports(10)

        self._imu.out.link(self._imu_xlink_out.input)

        # publisher
        self.imu_pub = self.create_publisher(
            Imu,
            "imu",
            10
        )
        self.imu_time = None

        # logging
        self.logging_imu_fd = None

    def init_rgb_camera(self):
        # params
        self.declare_parameter('rgb_camera_timeout', 5.0)
        self.declare_parameter('rgb_camera_frame','oakd_rgb')
        self.declare_parameter('rgb_camera_fps', 30.0)
        self.declare_parameter('rgb_camera_width', 640)
        self.declare_parameter('rgb_camera_height', 480)
        self.declare_parameter('publish_rgb_raw', True)
        self.declare_parameter('enable_rgb_uvc', False)
        self.declare_parameter('rgb_uvc_device_name', "/dev/video10")
        self.declare_parameter('rgb_encoder_bitrate_kbps', 512)
        self.declare_parameter('enable_rgb_camera_isp_scale', True)
        self.declare_parameter('rgb_camera_isp_scale', [4, 9])
        self.rgb_camera_timeout = self.get_parameter(
            'rgb_camera_timeout').get_parameter_value().double_value
        self.rgb_camera_frame = self.get_parameter(
            'rgb_camera_frame').get_parameter_value().string_value
        self.rgb_camera_fps = self.get_parameter(
            'rgb_camera_fps').get_parameter_value().double_value
        self.rgb_camera_width = self.get_parameter(
            'rgb_camera_width').get_parameter_value().integer_value
        self.rgb_camera_height = self.get_parameter(
            'rgb_camera_height').get_parameter_value().integer_value
        self.publish_rgb_raw = self.get_parameter(
            'publish_rgb_raw').get_parameter_value().bool_value
        self.enable_rgb_uvc = self.get_parameter(
            'enable_rgb_uvc').get_parameter_value().bool_value
        self.rgb_uvc_device_name = self.get_parameter(
            'rgb_uvc_device_name').get_parameter_value().string_value
        self.rgb_encoder_bitrate_kbps = self.get_parameter(
            'rgb_encoder_bitrate_kbps').get_parameter_value().integer_value
        self.enable_rgb_camera_isp_scale = self.get_parameter(
            'enable_rgb_camera_isp_scale').get_parameter_value().bool_value
        self.rgb_camera_isp_scale = self.get_parameter(
            'rgb_camera_isp_scale').get_parameter_value().integer_array_value

        # add Rgb camera to pipeline
        self._rgb_camera_xlink_out = self._pipeline.create(dai.node.XLinkOut)
        self._rgb_camera_xlink_out.setStreamName("rgb_camera")

        # Add video Encoder Node 
        self._rgb_camera_videoEnc = self._pipeline.create(dai.node.VideoEncoder)
        self._rgb_camera_videoEnc.setDefaultProfilePreset(
            self.rgb_camera_fps, dai.VideoEncoderProperties.Profile.H264_HIGH)
        self._rgb_camera_videoEnc.setBitrateKbps(
            self.rgb_encoder_bitrate_kbps) # Directly affects recording quality and size
        
        # Create XLINK out node for encoded stream 
        self._rgb_camera_xlink_enc_out = self._pipeline.create(dai.node.XLinkOut)
        self._rgb_camera_xlink_enc_out.setStreamName("rgb_enc_h264")

        self._rgb_camera = self._pipeline.create(dai.node.ColorCamera)
        self._rgb_camera.setInterleaved(False)
        if self.enable_rgb_camera_isp_scale:
            assert len(self.rgb_camera_isp_scale) == 2,\
                  "length of isp scale array needs to be 2. [numerator, denomenator]"
            self._rgb_camera.setIspScale(
                self.rgb_camera_isp_scale[0],
                self.rgb_camera_isp_scale[1]
            )
        self._rgb_camera.setColorOrder(dai.ColorCameraProperties.ColorOrder.RGB)
        self._rgb_camera.initialControl.setManualFocus(130)
        self._rgb_camera.setPreviewSize(self.rgb_camera_width, self.rgb_camera_height)
        self._rgb_camera.setVideoSize(self.rgb_camera_width, self.rgb_camera_height)
        self._rgb_camera.setFps(self.rgb_camera_fps)

        self._rgb_camera.preview.link(self._rgb_camera_xlink_out.input)
        
        # Link camera video to video encoder and add encoded stream to pipeline
        self._rgb_camera.video.link(self._rgb_camera_videoEnc.input)
        self._rgb_camera_videoEnc.bitstream.link(self._rgb_camera_xlink_enc_out.input)

        if self.enable_rgb_uvc:
            self.puvc = pyvirtualcam.Camera(
                width=self.rgb_camera_width, 
                height=self.rgb_camera_height, 
                fps=self.rgb_camera_fps,
                fmt=pyvirtualcam.PixelFormat.BGR,
                device=self.rgb_uvc_device_name
            )
        # publisher
        if self.publish_rgb_raw:
            self.rgb_camera_pub = self.create_publisher(
                Image,
                "oakd/rgb/image_raw",
                10
            )
        self.rgb_camera_time = None

        self.rgb_camera_intrinsics = {}

        # logging
        self.logging_rgb_fd = None
        self.logging_rgb_time_fd = None
        self.logging_rgb_calib_fd = None

    def init_depth_camera(self):
        # params
        self.declare_parameter('depth_camera_timeout', 5.0)
        self.declare_parameter('depth_camera_frame','oakd_depth')
        self.declare_parameter('depth_camera_fps', 10.0)
        self.depth_camera_timeout = self.get_parameter(
            'depth_camera_timeout').get_parameter_value().double_value
        self.depth_camera_frame = self.get_parameter(
            'depth_camera_frame').get_parameter_value().string_value
        self.depth_camera_fps = self.get_parameter(
            'depth_camera_fps').get_parameter_value().double_value

        # add depth camera to pipeline
        self._depth_camera_xlink_out = self._pipeline.create(dai.node.XLinkOut)
        self._depth_camera_xlink_out.setStreamName("depth_camera")

        self._mono_left = self._pipeline.create(dai.node.MonoCamera)
        self._mono_left.setFps(self.depth_camera_fps)
        self._mono_left.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        self._mono_left.setCamera("left")

        self._mono_right = self._pipeline.create(dai.node.MonoCamera)
        self._mono_right.setFps(self.depth_camera_fps)
        self._mono_right.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        self._mono_right.setCamera("right")

        self._depth_camera = self._pipeline.create(dai.node.StereoDepth)
        self._depth_camera.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_ACCURACY)
        self._depth_camera.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
        self._depth_camera.setLeftRightCheck(True)
        self._depth_camera.setExtendedDisparity(False)
        self._depth_camera.setSubpixel(False)

        self._mono_left.out.link(self._depth_camera.left)
        self._mono_right.out.link(self._depth_camera.right)
        self._depth_camera.disparity.link(self._depth_camera_xlink_out.input)

        # publisher
        self.depth_camera_pub = self.create_publisher(
            Image,
            "oakd/depth/image_raw",
            10
        )
        self.depth_camera_time = None

        self.depth_camera_intrinsics = {}

        # logging
        self.logging_depth_fd = None
        self.logging_depth_time_fd = None
        self.logging_depth_calib_fd = None

    def start_logging_cb(self, req, res):
        if not self.logging:
            self.logging_usage_fd = open(os.path.join(req.path_to_data_dir, "usage.csv"), "w")
            usage_header = "timestamp,ddr_ram_used,ddr_ram_total,"+\
                "cmx_ram_used,cmx_ram_total,"+\
                "leon_css_heap_used,leon_css_heap_total,"+\
                "leon_mss_heap_used,leon_mss_heap_total,"+\
                "average_temp,css_temp,mss_temp,upa_temp,dss_temp,"+\
                "leon_css_cpu_usage,leon_mss_cpu_usage\n"
            self.logging_usage_fd.write(usage_header)

            if self.enable_imu:
                self.logging_imu_fd = open(os.path.join(req.path_to_data_dir, "imu.csv"), "w")
                self.logging_imu_fd.write("timestamp,ax,ay,az,wx,wy,wz\n")
            if self.enable_rgb_camera:
                self.logging_rgb_camera_fd = \
                    av.open(os.path.join(req.path_to_data_dir, "oakd_rgb.mp4"), 'w')
                self.stream = self.logging_rgb_camera_fd.add_stream(
                    "h264", rate=self.rgb_camera_fps)
                self.stream.time_base = Fraction(1, 1000 * 1000 * 1000) # Nano
                self.logging_rgb_camera_timestamp_fd = open(
                    os.path.join(req.path_to_data_dir, "oakd_rgb_timstamps.csv"),
                    "w"
                )
                self.logging_rgb_camera_timestamp_fd.write("timestamp\n")
                self.logging_rgb_camera_intrinsics_fd = open(
                    os.path.join(req.path_to_data_dir, "oakd_rgb_intrinsics.yaml"),
                    "w"
                )
            if self.enable_depth_camera and self.enable_depth_camera_logging:
                self.logging_depth_camera_fd = cv2.VideoWriter(
                    os.path.join(req.path_to_data_dir, "oakd_depth.mp4"),
                    cv2.VideoWriter_fourcc(*'H264'),
                    self.depth_camera_fps,
                    (640, 400),
                    isColor=False
                )
                self.logging_depth_camera_timestamp_fd = open(
                    os.path.join(req.path_to_data_dir, "oakd_depth_timstamps.csv"),
                    "w"
                )
                self.logging_depth_camera_timestamp_fd.write("timestamp\n")
                self.logging_depth_camera_intrinsics_fd = open(
                    os.path.join(req.path_to_data_dir, "oakd_depth_intrinsics.yaml"),
                    "w"
                )
            self.logging = True
            res.success = True
        else:
            res.success = False
            res.message = "Already logging!"
        return res

    def stop_logging_cb(self, req, res):
        if self.logging:
            self.stop_logging()
            res.success = True
        else:
            res.success = False
            res.message = "Already not logging!"
        return res

    def stop_logging(self):
        self.logging_lock.acquire()
        if self.enable_imu:
            self.logging_imu_fd.close()
        if self.enable_rgb_camera:
            self.logging_rgb_camera_fd.close()
            self.logging_rgb_camera_timestamp_fd.close()
            yaml.dump(self.rgb_camera_intrinsics, self.logging_rgb_camera_intrinsics_fd)
            self.logging_rgb_camera_intrinsics_fd.close()
            self.rgb_camera_enc_start = None
        if self.enable_depth_camera and self.enable_depth_camera_logging:
            self.logging_depth_camera_fd.release()
            self.logging_depth_camera_timestamp_fd.close()
            yaml.dump(self.depth_camera_intrinsics, self.logging_depth_camera_intrinsics_fd)
            self.logging_depth_camera_intrinsics_fd.close()

        self.logging_usage_fd.close()
        self.logging = False
        self.logging_lock.release()

    def observer_cb(self):
        # 0. wait for every thing to boot up befor observing
        dt = (self.get_clock().now() - self.start_time).nanoseconds / 10**9
        if dt < 5.0:
            return

        # 0. observe usage
        if self.usage_time is not None:
            dt = (self.get_clock().now() - self.usage_time).nanoseconds / 10**9
            if dt >= self.usage_timeout:
                self.is_oakd_connected = False
                self.get_logger().warn(
                    f'No usage msg received since {dt:.4} sec')
                if self.logging:
                    self.stop_logging()
                rclpy.shutdown()
        else:
            self.get_logger().warn('No usage msg received yet!')

        # 1. observe imu
        if self.enable_imu:
            if self.imu_time is not None:
                dt = (self.get_clock().now() - self.imu_time).nanoseconds / 10**9
                if dt >= self.imu_timeout:
                    self.get_logger().warn(
                        f'No imu msg received since {dt:.4} sec')
            else:
                self.get_logger().warn('No imu msg received yet!')

        # 2. observe rgb camera
        if self.enable_rgb_camera:
            if self.rgb_camera_time is not None:
                dt = (self.get_clock().now() - self.rgb_camera_time).nanoseconds / 10**9
                if dt >= self.rgb_camera_timeout:
                    self.get_logger().warn(
                        f'No rgb camera frames received since {dt:.4} sec')
            else:
                self.get_logger().warn('No RGB frames received yet!')

        # 3. observe depth camera
        if self.enable_depth_camera:
            if self.depth_camera_time is not None:
                dt = (self.get_clock().now() - self.depth_camera_time).nanoseconds / 10**9
                if dt >= self.depth_camera_timeout:
                    self.get_logger().warn(
                        f'No depth camera frames received since {dt:.4} sec')
            else:
                self.get_logger().warn('No depth camera frames received yet!')

    def usage_cb(self, info):
        self.usage_time = self.get_clock().now()

        usage_time = self.get_clock().now()

        usage_msg = OakdUsage()
        usage_msg.header.stamp = usage_time.to_msg()
        usage_msg.header.frame_id = "oakd"

        usage_msg.ddr_ram_used = info.ddrMemoryUsage.used / BYTES_TO_MB
        usage_msg.ddr_ram_total = info.ddrMemoryUsage.total / BYTES_TO_MB

        usage_msg.cmx_ram_used = info.cmxMemoryUsage.used / BYTES_TO_MB
        usage_msg.cmx_ram_total = info.cmxMemoryUsage.total / BYTES_TO_MB

        usage_msg.leon_css_heap_used = info.leonCssMemoryUsage.used / BYTES_TO_MB
        usage_msg.leon_css_heap_total = info.leonCssMemoryUsage.total / BYTES_TO_MB

        usage_msg.leon_mss_heap_used = info.leonMssMemoryUsage.used / BYTES_TO_MB
        usage_msg.leon_mss_heap_total = info.leonMssMemoryUsage.total / BYTES_TO_MB

        temp = info.chipTemperature
        usage_msg.average_temp = temp.average
        usage_msg.css_temp = temp.css
        usage_msg.mss_temp = temp.mss
        usage_msg.upa_temp = temp.upa
        usage_msg.dss_temp = temp.dss

        usage_msg.leon_css_cpu_usage = info.leonCssCpuUsage.average * 100
        usage_msg.leon_mss_cpu_usage = info.leonMssCpuUsage.average * 100

        self.oakd_usage_pub.publish(usage_msg)

        self.logging_lock.acquire()
        if self.logging:
            self.logging_usage_fd.write(
                f"{usage_time.nanoseconds/1e9},"+\
                f"{usage_msg.ddr_ram_used},"+\
                f"{usage_msg.ddr_ram_total},"+\
                f"{usage_msg.cmx_ram_used},"+\
                f"{usage_msg.cmx_ram_total},"+\
                f"{usage_msg.leon_css_heap_used},"+\
                f"{usage_msg.leon_css_heap_total},"+\
                f"{usage_msg.leon_mss_heap_used},"+\
                f"{usage_msg.leon_mss_heap_total},"+\
                f"{usage_msg.average_temp},"+\
                f"{usage_msg.css_temp},"+\
                f"{usage_msg.mss_temp},"+\
                f"{usage_msg.upa_temp},"+\
                f"{usage_msg.dss_temp},"+\
                f"{usage_msg.leon_css_cpu_usage},"+\
                f"{usage_msg.leon_mss_cpu_usage}\n"
            )
        self.logging_lock.release()

    def imu_cb(self, imu_data):
        linear_acceleration = imu_data.acceleroMeter
        angular_velocity = imu_data.gyroscope
        quaternion = imu_data.rotationVector

        self.imu_time = self.get_clock().now()

        imu_msg = Imu()
        imu_msg.header.stamp = self.imu_time.to_msg()
        imu_msg.header.frame_id = "imu"

        imu_msg.linear_acceleration.x = linear_acceleration.x
        imu_msg.linear_acceleration.y = linear_acceleration.y
        imu_msg.linear_acceleration.z = linear_acceleration.z
        imu_msg.linear_acceleration_covariance[0] = \
            self.imu_acceleration_covariance_diag[0]
        imu_msg.linear_acceleration_covariance[4] = \
            self.imu_acceleration_covariance_diag[1]
        imu_msg.linear_acceleration_covariance[8] = \
            self.imu_acceleration_covariance_diag[2]

        imu_msg.angular_velocity.x = angular_velocity.x
        imu_msg.angular_velocity.y = angular_velocity.y
        imu_msg.angular_velocity.z = angular_velocity.z
        imu_msg.angular_velocity_covariance[0] = \
            self.imu_angular_velocity_covariance_diag[0]
        imu_msg.angular_velocity_covariance[4] = \
            self.imu_angular_velocity_covariance_diag[1]
        imu_msg.angular_velocity_covariance[8] = \
            self.imu_angular_velocity_covariance_diag[2]

        imu_msg.orientation.w = quaternion.real
        imu_msg.orientation.x = quaternion.i
        imu_msg.orientation.y = quaternion.j
        imu_msg.orientation.z = quaternion.k
        imu_msg.orientation_covariance[0] = \
            self.imu_orientation_covariance_diag[0]
        imu_msg.orientation_covariance[4] = \
            self.imu_orientation_covariance_diag[1]
        imu_msg.orientation_covariance[8] = \
            self.imu_orientation_covariance_diag[2]

        self.imu_pub.publish(imu_msg)

        self.logging_lock.acquire()
        if self.logging:
            self.logging_imu_fd.write(
                f"{self.imu_time.nanoseconds/1e9},"+\
                f"{imu_msg.linear_acceleration.x},"+\
                f"{imu_msg.linear_acceleration.y},"+\
                f"{imu_msg.linear_acceleration.z},"+\
                f"{imu_msg.angular_velocity.x},"+\
                f"{imu_msg.angular_velocity.y},"+\
                f"{imu_msg.angular_velocity.z}\n"
            )
        self.logging_lock.release()

    def rgb_camera_cb(self,rgb_camera_data):
        self.rgb_camera_time = self.get_clock().now()

        rgb_camera_image = rgb_camera_data.getCvFrame()
        if self.enable_rgb_uvc:
            self.puvc.send(rgb_camera_image)
 
        if not self.publish_rgb_raw:
            return

        rgb_camera_cv2_image = rgb_camera_data.getCvFrame()
        image_raw_msgs = self.cv_bridge.cv2_to_imgmsg(rgb_camera_cv2_image, 'bgr8')
        image_raw_msgs.header.stamp = self.rgb_camera_time.to_msg()
        image_raw_msgs.header.frame_id = self.rgb_camera_frame
        self.rgb_camera_pub.publish(image_raw_msgs)

    def rgb_camera_enc_cb(self, rgb_enc_camera_data):
        self.logging_lock.acquire()
        if self.logging:
            if self.rgb_camera_enc_start is None:
                self.rgb_camera_enc_start = self.get_clock().now()
                self.logging_lock.release()
                return
            # Encoding data to packets with indiviual timestamps
            rgb_enc_data = rgb_enc_camera_data.getData()
            rgb_enc_data_packet = av.Packet(rgb_enc_data) 
            # Set frame timestamp
            rgb_enc_data_packet.pts = \
                (self.get_clock().now() - self.rgb_camera_enc_start).nanoseconds
            
            self.logging_rgb_camera_fd.mux_one(rgb_enc_data_packet)
            self.logging_rgb_camera_timestamp_fd.write(
                f"{self.rgb_camera_time.nanoseconds/1e9},\n"
            )
        self.logging_lock.release()

    def depth_camera_cb(self, depth_camera_data):
        self.depth_camera_time = self.get_clock().now()

        depth_camera_image = depth_camera_data.getCvFrame()

        image_raw_msgs = self.cv_bridge.cv2_to_imgmsg(depth_camera_image, 'passthrough')
        image_raw_msgs.header.stamp = self.depth_camera_time.to_msg()
        image_raw_msgs.header.frame_id = self.depth_camera_frame

        self.depth_camera_pub.publish(image_raw_msgs)

        if self.enable_depth_camera_logging:
            self.logging_lock.acquire()
            if self.logging:
                self.logging_depth_camera_fd.write(depth_camera_image)
                self.logging_depth_camera_timestamp_fd.write(
                    f"{self.depth_camera_time.nanoseconds/1e9},\n"
                )
            self.logging_lock.release()

    def start_oakd(self):
        with dai.Device(self._pipeline) as device:
            self.device = device
            self.usage_queue = device.getOutputQueue(
                    name="usage", maxSize=1, blocking=False)

            usage_thread = threading.Thread(target=self.usage_thread_cb, daemon=True)
            usage_thread.start()

            if self.enable_imu:
                imu_type = device.getConnectedIMU()
                imu_firmware_version = device.getIMUFirmwareVersion()
                self.get_logger().info(f'MxId: {device.getDeviceInfo().getMxId()}')
                self.get_logger().info(f'USB speed: {device.getUsbSpeed()}')
                self.get_logger().info(
                    f"IMU sensor: {imu_type}, firmware version: {imu_firmware_version}"
                )
                self.get_logger().info(f"Starting OAK-D IMU {imu_type} at {self.imu_hz}Hz")

                self.imu_queue = device.getOutputQueue(
                    name="imu", maxSize=1, blocking=False)

                imu_thread = threading.Thread(target=self.imu_thread, daemon=True)
                imu_thread.start()

            if self.enable_rgb_camera:
                camera_type = device.getConnectedCameras()
                self.get_logger().info(f"Connected cameras: {camera_type}")
                self.get_logger().info(
                    f"Starting OAK-D cameras {camera_type} at {self.rgb_camera_fps}Hz"
                )
                self.rgb_camera_queue = device.getOutputQueue(
                    name="rgb_camera", maxSize=1, blocking=False)

                # Get Encoded stream
                self.rgb_enc_stream = device.getOutputQueue(
                    name="rgb_enc_h264", maxSize=30, blocking=False)
                self.rgb_camera_enc_start = None

                calibrations = device.readCalibration()
                K = calibrations.getCameraIntrinsics(
                    dai.CameraBoardSocket.CAM_A,
                    (self.rgb_camera_width, self.rgb_camera_height)
                )
                distortion_parameters = calibrations.getDistortionCoefficients(
                    dai.CameraBoardSocket.CAM_A)
                distortion_model = calibrations.getDistortionModel(
                    dai.CameraBoardSocket.CAM_A).name
                fov = calibrations.getFov(dai.CameraBoardSocket.CAM_A)
                self.rgb_camera_intrinsics = {
                    "width": self.rgb_camera_width,
                    "height": self.rgb_camera_height,
                    "fov": fov,
                    "K": list(K),
                    "distortion_model": distortion_model,
                    "distortion_parameters": list(distortion_parameters)
                }

                rgb_camera_thread = threading.Thread(target=self.rgb_camera_thread, daemon=True)
                rgb_camera_thread.start()

                rgb_camera_enc_thread=threading.Thread(target=self.rgb_camera_enc_thread_cb,daemon=True)
                rgb_camera_enc_thread.start()

            if self.enable_depth_camera:
                self.depth_camera_queue = device.getOutputQueue(
                    name="depth_camera", maxSize=1, blocking=False)

                calibrations = device.readCalibration()
                K = calibrations.getCameraIntrinsics(
                    dai.CameraBoardSocket.CAM_B,
                    (640, 400)
                )
                distortion_parameters = calibrations.getDistortionCoefficients(
                    dai.CameraBoardSocket.CAM_B)
                distortion_model = calibrations.getDistortionModel(
                    dai.CameraBoardSocket.CAM_B).name
                fov = calibrations.getFov(dai.CameraBoardSocket.CAM_B)
                baseline = calibrations.getBaselineDistance(
                    dai.CameraBoardSocket.CAM_B,
                    dai.CameraBoardSocket.CAM_C
                )/1e2
                self.depth_camera_intrinsics = {
                    "width": 640,
                    "height": 400,
                    "fov": fov,
                    "K": list(K),
                    "distortion_model": distortion_model,
                    "distortion_parameters": list(distortion_parameters),
                    "baseline": baseline
                }


                depth_camera_thread = threading.Thread(target=self.depth_camera_thread, daemon=True)
                depth_camera_thread.start()

            while rclpy.ok():
                usage_thread.join(timeout=1)
                if self.enable_imu:
                    imu_thread.join(timeout=1)
                if self.enable_rgb_camera:
                    rgb_camera_thread.join(timeout=1)
                    rgb_camera_enc_thread.join(timeout=1)
                if self.enable_depth_camera:
                    depth_camera_thread.join(timeout=1)
                time.sleep(1)

            if self.enable_rgb_uvc:
                self.puvc.close()

    def usage_thread_cb(self):
        while rclpy.ok():
            try:
                sys_info = self.usage_queue.get()
                self.usage_cb(sys_info)
            except Exception as e:
                print(f"Exception in usage_thread (i.e. {e})")
                break

    def imu_thread(self):
        while rclpy.ok():
            try:
                imu_data = self.imu_queue.get()
                for data in imu_data.packets:
                    self.imu_cb(data)
            except Exception as e:
                print(f"Exception in imu_thread (i.e. {e})")
                break

    def rgb_camera_thread(self):
        while rclpy.ok():
            try:
                rgb_camera_data = self.rgb_camera_queue.get()
                self.rgb_camera_cb(rgb_camera_data)
            except Exception as e:
                print(f"Exception in rgb_camera_thread (i.e. {e})")
                break

    def rgb_camera_enc_thread_cb(self):
        while rclpy.ok():
            try:
                rgb_enc_camera_data = self.rgb_enc_stream.get()
                self.rgb_camera_enc_cb(rgb_enc_camera_data)
            except Exception as e:
                print(f"Exception in rgb_camera_enc_thread (i.e. {e})")
                break

    def depth_camera_thread(self):
        while rclpy.ok():
            try:
                depth_camera_data = self.depth_camera_queue.get()
                self.depth_camera_cb(depth_camera_data)
            except Exception as e:
                print(f"Exception in depth_camera_thread (i.e. {e})")
                break
                    

def main(args=None):
    rclpy.init(args=args)
    oakd_driver = OakdDriver()
    multi_thread_executor = MultiThreadedExecutor()
    multi_thread_executor.add_node(oakd_driver)
    try:
        oakd_thread = threading.Thread(target=oakd_driver.start_oakd)
        oakd_thread.start()
        spin(oakd_driver, executor=multi_thread_executor, hz=60)
    except KeyboardInterrupt:
        print("Killing oakd_driver!")
        oakd_driver.stop_logging_cb(StopLogging.Request(), StopLogging.Response())
        oakd_thread.join()
        oakd_driver.destroy_node()


if __name__ == "__main__":
    main()
