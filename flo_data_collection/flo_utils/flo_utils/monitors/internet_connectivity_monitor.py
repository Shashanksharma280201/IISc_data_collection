#!/usr/bin/env python3

import os
import time
from threading import Lock, Thread

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from flo_msgs.msg import PingResult
from flo_msgs.srv import StartLogging, StopLogging
from icmplib import multiping

from flo_utils.ros2_utils import spin


class InternetConnectivityMonitor(Node):
    def __init__(self):
        super().__init__('internet_connectivity_monitor')
        self.hosts_to_ping = self.declare_parameter(
            'hosts_to_ping', ['8.8.8.8', '1.1.1.1']).value
        self.ping_count = self.declare_parameter('ping_count', 5).value
        self.ping_interval = self.declare_parameter('ping_interval', 0.2).value
        self.ping_timeout = self.declare_parameter('ping_timeout', 2.0).value
        self.concurrent_tasks = self.declare_parameter(
            'concurrent_tasks', 50).value
        self.payload_size = self.declare_parameter('payload_size', 56).value

        self.ping_result_pub = self.create_publisher(
            PingResult,
            "/internet_connectivity_monitor/ping_results",
            10
        )

        self.logging = False
        self.logging_fd = None
        self.logging_lock = Lock()
        self.start_logging = self.create_service(
            StartLogging,
            'internet_connectivity_monitor/start_logging',
            self.start_logging_cb
        )
        self.stop_logging = self.create_service(
            StopLogging,
            'internet_connectivity_monitor/stop_logging',
            self.stop_logging_cb
        )

        self.ping_thread = Thread(target=self.ping_cb, daemon=True)
        self.ping_thread.start()
        self.get_logger().info("Started internet connectivity monitor")

    def start_logging_cb(self, req, res):
        if not self.logging:
            self.logging_fd = open(os.path.join(
                req.path_to_data_dir, "ping_results.csv"), "w")
            csv_header = \
                "timestamp,host,min_rtt,max_rtt,avg_rtt,packets_sent,packets_received,packet_loss,jitter"
            self.logging_fd.write(
                f"{csv_header}\n"
            )
            self.logging = True
            res.success = True
        else:
            res.success = False
            res.message = "Already logging!"
        return res

    def stop_logging_cb(self, _, res):
        if self.logging:
            self.logging_lock.acquire()
            self.logging_fd.close()
            self.logging = False
            self.logging_lock.release()
            res.success = True
        else:
            res.success = True
            res.message = "Already not logging!"
        return res
    
    def parse_ping_result(self, result):
        msg = PingResult()
        msg.header.frame_id = result.address

        msg.address = result.address
        msg.is_alive = result.is_alive
        
        msg.min_rtt = result.min_rtt
        msg.max_rtt = result.max_rtt
        msg.avg_rtt = result.avg_rtt
        msg.rtts = result.rtts

        msg.packets_sent = result.packets_sent
        msg.packets_received = result.packets_received
        msg.packet_loss = result.packet_loss
        
        msg.jitter = result.jitter
        
        return msg

    def ping_cb(self):
        while rclpy.ok():
            ping_results = multiping(
                addresses=self.hosts_to_ping,
                count=self.ping_count,
                interval=self.ping_interval,
                concurrent_tasks=self.concurrent_tasks,
                privileged=False,
                timeout=self.ping_timeout,
                payload_size=self.payload_size
            )
            for entry in ping_results:
                # self.get_logger().info(f"\n\n{entry}")
                stamp = self.get_clock().now()
                msg = self.parse_ping_result(entry)
                msg.header.stamp = stamp.to_msg()

                self.ping_result_pub.publish(msg)

                self.logging_lock.acquire()
                if self.logging:
                    self.logging_fd.write(
                        f"{stamp.nanoseconds/1e9},"+\
                        f"{msg.address},"+\
                        f"{msg.min_rtt},"+\
                        f"{msg.max_rtt},"+
                        f"{msg.avg_rtt},"+
                        f"{msg.packets_sent},"+
                        f"{msg.packets_received},"+
                        f"{msg.packet_loss},"+
                        f"{msg.jitter}\n"
                    )
                self.logging_lock.release()
            
            time.sleep(1.0)


def main(args=None):
    rclpy.init(args=args)
    internet_connectivity_monitor = InternetConnectivityMonitor()
    multi_thread_executor = MultiThreadedExecutor()
    multi_thread_executor.add_node(internet_connectivity_monitor)
    try:
        spin(internet_connectivity_monitor,
             executor=multi_thread_executor, hz=125)
    except KeyboardInterrupt:
        print("Killing internet_connectivity_monitor!")
        internet_connectivity_monitor.stop_logging_cb(
            StopLogging.Request(), StopLogging.Response())
        internet_connectivity_monitor.destroy_node()


if __name__ == "__main__":
    main()
