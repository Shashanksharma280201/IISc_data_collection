#!/usr/bin/env python3

import os
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

from threading import Lock
from std_msgs.msg import UInt8MultiArray

from flo_msgs.srv import StartLogging, StopLogging
from flo_utils.utils import pretty_print_time
from flo_utils.ros2_utils import spin

class RtcmMonitor(Node):
    def __init__(self):
        super().__init__('rtcm_monitor')

        self.declare_parameter('rtcm_timeout', 15.0)
        self.rtcm_timeout = self.get_parameter(
            'rtcm_timeout').get_parameter_value().double_value

        self.rtcm_sub = self.create_subscription(
            UInt8MultiArray,
            'rtcm/out',
            self.rtcm_cb,
            callback_group=ReentrantCallbackGroup(),
            qos_profile=10)
        
        # logging
        self.logging = False
        self.logging_lock = Lock()
        self.start_logging_service = self.create_service(
            StartLogging,
            'rtcm_monitor/start_logging',
            self.start_logging_cb
        )
        self.stop_logging_service = self.create_service(
            StopLogging,
            'rtcm_monitor/stop_logging',
            self.stop_logging_cb
        )

        self.observer_timer = self.create_timer(60, self.observer_cb)
        self.rtcm_msg_count = 0
        self.last_msg_time = None
        self.last_warning_time = None

    def start_logging_cb(self, req, res):
        if not self.logging:
            self.logging_rtcm_fd = open(os.path.join(req.path_to_data_dir, "rtcm.csv"), "w")
            self.logging_rtcm_fd.write("timestamp,msgs_in_window\n")
            self.logging = True
            res.success = True
        else:
            res.success = False
            res.message = "Already logging!"
        return res
    
    def stop_logging_cb(self, req, res):
        if self.logging:
            self.logging_lock.acquire()
            self.logging_rtcm_fd.close()
            self.logging = False
            self.logging_lock.release()
            res.success = True
        else:
            res.success = False
            res.message = "Already not logging!"
        return res
    
    def rtcm_cb(self, msg):
        self.rtcm_msg_count +=1
        self.last_msg_time = self.get_clock().now()
        
    def observer_cb(self):
        if self.last_msg_time is None:
            self.get_logger().warn('RTCM data not yet received')
            return
        
        dt = (self.get_clock().now() - self.last_msg_time).nanoseconds / 10**9
        if dt >= self.rtcm_timeout:
            self.handle_warning(dt)
        else:
            self.last_warning_time = None

        self.logging_lock.acquire()
        if self.logging:
            self.logging_rtcm_fd.write(
                f"{self.get_clock().now().nanoseconds/1e9}," +
                f"{self.rtcm_msg_count}\n"
            )
        self.logging_lock.release()
        self.rtcm_msg_count = 0

    def handle_warning(self, dt):
        if self.last_warning_time is None:
            self.last_warning_time = self.get_clock().now()
            self.get_logger().warn(f"RTCM msgs not received for {pretty_print_time(dt)}")
            return

        dt_w = (self.get_clock().now() -
                self.last_warning_time).nanoseconds / 1e9
        if dt_w >= (1.0 * self.get_warning_multiplier(dt)):
            self.last_warning_time = self.get_clock().now()
            self.get_logger().warn(f"RTCM msgs not received for {pretty_print_time(dt)}")

    def get_warning_multiplier(self, dt):
        if dt > 120.0:
            return 180.0
        elif dt > 60.0:
            return 60.0
        elif dt > 15.0:
            return 15.0
        else:
            return 1.0


def main(args=None):
    rclpy.init(args=args)
    rtcm_monitor = RtcmMonitor()
    multi_thread_executor = MultiThreadedExecutor()
    multi_thread_executor.add_node(rtcm_monitor)
    try:
        spin(rtcm_monitor, executor=multi_thread_executor, hz=125)
    except KeyboardInterrupt:
        print("Killing rtcm monitor ..!")
        rtcm_monitor.stop_logging_cb(
            StopLogging.Request(), StopLogging.Response())
        rtcm_monitor.destroy_node()


if __name__ == "__main__":
    main()