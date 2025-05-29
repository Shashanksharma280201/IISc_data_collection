#!/usr/bin/env python3

import sys
import rclpy
from rclpy.node import Node

from std_msgs.msg import UInt8MultiArray
from rclpy.executors import MultiThreadedExecutor
from flo_utils.ntrip_client import NTRIPClient
from flo_utils.ros2_utils import spin, Rate


class NTRIPRos(Node):
    def __init__(self):
        super().__init__('ntrip_client')
        self.declare_parameter('_debug', False)
        self._debug = self.get_parameter(
            '_debug'
        ).get_parameter_value().bool_value

        self.declare_parameter('host', 'rtk2go.com')
        self.host = self.get_parameter(
            'host'
        ).get_parameter_value().string_value

        self.declare_parameter('port', 2101)
        self.port = self.get_parameter(
            'port'
        ).get_parameter_value().integer_value

        self.declare_parameter('mountpoint', 'FloMobilityBase')
        self.mountpoint = self.get_parameter(
            'mountpoint'
        ).get_parameter_value().string_value

        self.declare_parameter('ntrip_version', 1.0)
        self.ntrip_version = self.get_parameter(
            'ntrip_version'
        ).get_parameter_value().double_value

        self.declare_parameter('authenticate', True)
        self.authenticate = self.get_parameter(
            'authenticate'
        ).get_parameter_value().bool_value

        self.declare_parameter('username', 'pratham@flomobility.com')
        self.username = self.get_parameter(
            'username'
        ).get_parameter_value().string_value

        self.declare_parameter('password', 'FloStation123')
        self.password = self.get_parameter(
            'password'
        ).get_parameter_value().string_value

        self.declare_parameter('reconnect_time', 60.0)
        self.reconnect_time = self.get_parameter(
            'reconnect_time'
        ).get_parameter_value().double_value

        self.declare_parameter('ssl', False)
        self.ssl = self.get_parameter(
            'ssl'
        ).get_parameter_value().bool_value

        self.declare_parameter('cert', 'None')
        self.cert = self.get_parameter(
            'cert'
        ).get_parameter_value().string_value

        self.declare_parameter('key', 'None')
        self.key = self.get_parameter(
            'key'
        ).get_parameter_value().string_value

        self.declare_parameter('ca_cert', 'None')
        self.ca_cert = self.get_parameter(
            'ca_cert'
        ).get_parameter_value().string_value

        self.declare_parameter('reconnect_attempt_max',
                               NTRIPClient.DEFAULT_RECONNECT_ATTEMPT_MAX)
        self.reconnect_attempt_max = self.get_parameter(
            'reconnect_attempt_max'
        ).get_parameter_value().integer_value
        self.declare_parameter('reconnect_attempt_wait_seconds',
                               NTRIPClient.DEFAULT_RECONNECT_ATEMPT_WAIT_SECONDS)
        self.reconnect_attempt_wait_seconds = self.get_parameter(
            'reconnect_attempt_wait_seconds'
        ).get_parameter_value().integer_value

        self.declare_parameter('rtcm_timeout_seconds',
                               NTRIPClient.DEFAULT_RTCM_TIMEOUT_SECONDS)
        self.rtcm_timeout_seconds = self.get_parameter(
            'rtcm_timeout_seconds'
        ).get_parameter_value().integer_value

        if self.ntrip_version == 'None':
            self.ntrip_version = None

        if self._debug:
            rclpy.logging.set_logger_level(
                self.get_logger().name, rclpy.logging.LoggingSeverity.DEBUG)

        if self.authenticate:
            if not self.username:
                self.get_logger().error(
                    'Requested to authenticate, but param "username" was not set')
                sys.exit(1)
            if not self.password:
                self.get_logger().error(
                    'Requested to authenticate, but param "password" was not set')
                sys.exit(1)

        # publisher
        self._rtcm_pub = self.create_publisher(
            UInt8MultiArray, 'rtcm/out', 10)

        # Initialize the client
        self._client = NTRIPClient(
            host=self.host,
            port=self.port,
            mountpoint=self.mountpoint,
            ntrip_version=self.ntrip_version,
            username=self.username,
            password=self.password,
            logerr=self.get_logger().error,
            logwarn=self.get_logger().warning,
            loginfo=self.get_logger().info,
            logdebug=self.get_logger().debug
        )

        # Get some SSL parameters for the NTRIP client
        self._client.ssl = self.get_parameter('ssl').value
        self._client.cert = self.get_parameter('cert').value
        self._client.key = self.get_parameter('key').value
        self._client.ca_cert = self.get_parameter('ca_cert').value
        if self._client.cert == 'None':
            self._client.cert = None
        if self._client.key == 'None':
            self._client.key = None
        if self._client.ca_cert == 'None':
            self._client.ca_cert = None

        # Get some timeout parameters for the NTRIP client
        self._client.reconnect_attempt_max = self.get_parameter(
            'reconnect_attempt_max').value
        self._client.reconnect_attempt_wait_seconds = self.get_parameter(
            'reconnect_attempt_wait_seconds').value
        self._client.rtcm_timeout_seconds = self.get_parameter(
            'rtcm_timeout_seconds').value

        self.run()

    def run(self):
        rate = Rate(hz=1.0/self.reconnect_time)
        while rclpy.ok():
            if not self._client.connect():
                self.get_logger().error('Unable to connect to NTRIP server, '+\
                                        'retrying: check connectivity')
                # self.stop()
            else:
                self._rtcm_timer = self.create_timer(0.1, self.publish_rtcm)
                return True
            rate.sleep()
        return False

    def stop(self):
        self.get_logger().info('Stopping RTCM publisher')
        if self._rtcm_timer:
            self._rtcm_timer.cancel()
            self._rtcm_timer.destroy()
        self.get_logger().info('Disconnecting NTRIP client')
        self._client.disconnect()
        self.get_logger().info('Shutting down node')
        self.destroy_node()

    def publish_rtcm(self):
        for raw_rtcm in self._client.recv_rtcm():
            msg = UInt8MultiArray()
            msg.data = raw_rtcm
            self._rtcm_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    ntrip_client = NTRIPRos()
    # if not ntrip_client.run():
    #   sys.exit(1)
    multi_thread_executor = MultiThreadedExecutor()
    multi_thread_executor.add_node(ntrip_client)
    try:
        spin(ntrip_client, executor=multi_thread_executor, hz=125)
    except KeyboardInterrupt:
        ntrip_client.stop()


if __name__ == '__main__':
    main()
