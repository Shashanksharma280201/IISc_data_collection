#!/usr/bin/env python3

import os
from threading import Lock

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from rcl_interfaces.msg import Log

from flo_msgs.srv import StartLogging, StopLogging

from flo_utils.ros2_utils import spin


class RosoutMonitor(Node):
    def __init__(self):
        super().__init__('rosout_monitor')

        self.declare_parameter('logger_level', 20)
        self.logger_level = self.get_parameter(
            'logger_level'
        ).get_parameter_value().integer_value

        self.rosout_sub = self.create_subscription(
            Log,
            "rosout",
            self.rosout_cb,
            10
        )

        self.logger_level_map = {
            10: 'DEBUG',
            20: 'INFO',
            30: 'WARN',
            40: 'ERROR',
            50: 'FATAL'
        }

        self.logging = False
        self.logging_fd = None
        self.logging_lock = Lock()
        self.start_logging = self.create_service(
            StartLogging,
            'rosout_monitor/start_logging',
            self.start_logging_cb
        )
        self.stop_logging = self.create_service(
            StopLogging,
            'rosout_monitor/stop_logging',
            self.stop_logging_cb
        )

    def start_logging_cb(self, req, res):
        if not self.logging:
            self.logging_fd = open(os.path.join(req.path_to_data_dir, "rosout.logs"), "w")
            self.logging = True
            res.success = True
        else:
            res.success = True
            res.message = "Already logging!"
        return res

    def stop_logging_cb(self, req, res):
        if self.logging:
            self.logging_lock.acquire()
            self.logging_fd.close()
            self.logging = False
            self.logging_lock.release()
            res.success = True
        else:
            res.success = False
            res.message = "Already not logging!"
        return res

    def rosout_cb(self, msg):
        self.logging_lock.acquire()
        if self.logging:
            self.logging_fd.write(
                f"[{self.get_clock().now().nanoseconds/1e9}] "+\
                f"[Logger: {msg.name}] "+\
                f"[{self.logger_level_map[msg.level]}] "+\
                f"[file: {msg.file}] "+\
                f"[function: {msg.function}] "+\
                f"[line: {msg.line}] "+\
                f"{msg.msg}\n"
            )
        self.logging_lock.release()


def main(args=None):
    rclpy.init(args=args)
    rosout_monitor = RosoutMonitor()
    multi_thread_executor = MultiThreadedExecutor()
    multi_thread_executor.add_node(rosout_monitor)
    try:
        spin(rosout_monitor, executor=multi_thread_executor, hz=125)
    except KeyboardInterrupt:
        print("Killing rosout_monitor!")
        rosout_monitor.stop_logging_cb(StopLogging.Request(), StopLogging.Response())
        rosout_monitor.destroy_node()


if __name__ == "__main__":
    main()
