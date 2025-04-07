#!/usr/bin/env python3

import os
import math
from threading import Lock

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from std_msgs.msg import String

from flo_msgs.msg import JointCmd, JointState
from flo_msgs.srv import Enable, StartLogging, StopLogging

# from flo_utils.ros2_utils import spin


class FloBaseDiff(Node):
    def __init__(self):
        super().__init__('flo_base_diff')

        self.declare_parameter('debug', False)
        self.debug = self.get_parameter('debug').get_parameter_value().bool_value
        
        # serial devices
        self.serial_device_pub = self.create_publisher(
            String,
            "serial_port_drive/in",
            10
        )
        self.serial_device_sub = self.create_subscription(
            String,
            "serial_port_drive/out",
            self.serial_device_cb,
            10
        )

        self.hb_time = None # Time of last received HB
        self.declare_parameter('hb_timeout', 1.0)
        self.hb_timeout = self.get_parameter('hb_timeout').get_parameter_value().double_value

        self.declare_parameter('hb_hz', 10.0) # For publishing HB
        self.hb_hz = self.get_parameter('hb_hz').get_parameter_value().double_value
        self.hb_timer = self.create_timer(1.0/self.hb_hz, self.hb_cb)

        # base drive
        self.declare_parameter('left_wheel_joint_index', 0)
        self.declare_parameter('right_wheel_joint_index', 1)
        self.declare_parameter('wheel_diameter', 1.0)
        self.declare_parameter('wheel_to_wheel_separation', 1.0)
        self.declare_parameter('wheel_separation_multiplier', 1.0)
        self.declare_parameter('cmd_vel_timeout_enabled', False)
        self.declare_parameter('cmd_vel_timeout', 5.0)
        self.declare_parameter('publish_odom', False)
        self.declare_parameter('publish_pose_in_odom', False)
        self.declare_parameter('odom_hz', 30.0)
        self.declare_parameter('drive_joint_timeout', 1.0)
        self.declare_parameter('enable_observer', False)
        self.declare_parameter('observer_hz', 1.0)
        self.declare_parameter('pose_covariance_diagonal',
                               [1000000.0, 1000000.0, 0.000001, 0.000001, 0.000001, 1000000.0])
        self.declare_parameter('twist_covariance_diagonal',
                               [0.08, 0.000001, 0.0000001, 0.0000001, 0.0000001, 0.06])
        self.left_wheel_joint_index = self.get_parameter(
            'left_wheel_joint_index'
        ).get_parameter_value().integer_value
        self.right_wheel_joint_index = self.get_parameter(
            'right_wheel_joint_index'
        ).get_parameter_value().integer_value
        self.wheel_diameter = self.get_parameter(
            'wheel_diameter'
        ).get_parameter_value().double_value
        self.wheel_to_wheel_separation = self.get_parameter(
            'wheel_to_wheel_separation'
        ).get_parameter_value().double_value
        self.wheel_separation_multiplier = self.get_parameter(
            'wheel_separation_multiplier'
        ).get_parameter_value().double_value
        self.cmd_vel_timeout_enabled = self.get_parameter(
            'cmd_vel_timeout_enabled'
        ).get_parameter_value().bool_value
        self.cmd_vel_timeout = self.get_parameter(
            'cmd_vel_timeout'
        ).get_parameter_value().double_value
        self.publish_odom = self.get_parameter('publish_odom').get_parameter_value().bool_value
        self.publish_pose_in_odom = self.get_parameter(
            'publish_pose_in_odom').get_parameter_value().bool_value
        self.odom_hz = self.get_parameter('odom_hz').get_parameter_value().double_value
        self.drive_joint_timeout = self.get_parameter(
            'drive_joint_timeout'
        ).get_parameter_value().double_value
        self.enable_observer = self.get_parameter(
            'enable_observer').get_parameter_value().bool_value
        self.observer_hz = self.get_parameter('observer_hz').get_parameter_value().double_value
        self.pose_covariance_diagonal = self.get_parameter(
            'pose_covariance_diagonal').get_parameter_value().double_array_value
        self.twist_covariance_diagonal = self.get_parameter(
            'twist_covariance_diagonal').get_parameter_value().double_array_value

        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_cb,
            10
        )

        self.odom_pub = self.create_publisher(
            Odometry,
            "odom/wheel_encoder",
            10
        )
        self.odom_timer = self.create_timer(1.0/self.odom_hz, self.odom_cb)

        self.declare_parameter('enable', False)
        self.enable = self.get_parameter('enable').get_parameter_value().bool_value
        self.enable_service = self.create_service(
            Enable,
            'flo_base_diff/enable',
            self.enable_base_cb
        )

        self.logging = False
        self.logging_fd = None
        self.logging_lock = Lock()
        self.start_logging_service = self.create_service(
            StartLogging,
            'flo_base_diff/start_logging',
            self.start_logging_cb
        )
        self.stop_logging_service = self.create_service(
            StopLogging,
            'flo_base_diff/stop_logging',
            self.stop_logging_cb
        )

        if self.debug:
            self.left_joint_state_pub = self.create_publisher(
                JointState,
                "joints/state/left_wheel",
                10
            )

            self.right_joint_state_pub = self.create_publisher(
                JointState,
                "joints/state/right_wheel",
                10
            )

            self.left_joint_cmd_pub = self.create_publisher(
                JointCmd,
                "joints/cmd/left_wheel",
                10
            )

            self.right_joint_cmd_pub = self.create_publisher(
                JointCmd,
                "joints/cmd/right_wheel",
                10
            )

        self.wl = 0.0 # Left wheel angular velocity
        self.wr = 0.0 # Right wheel angular velocity

        self.odom_time = None
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        self.v_cmd = 0.0
        self.w_cmd = 0.0
        self.cmd_vel_time = None

        if self.enable_observer:
            self.start_time = self.get_clock().now()
            self.observer_timer = self.create_timer(1.0 / self.observer_hz, self.observer_cb)

    def serial_device_write(self, data):
        msg = String()
        msg.data = data
        self.serial_device_pub.publish(msg)

    def serial_device_cb(self, msg):
        data_split = msg.data.split(" ")
        if data_split[0] == 'HB': # Heartbeat
            self.hb_time = self.get_clock().now()
        elif data_split[0] == 'J': # Joint
            if len(data_split) != 5:
                return
            try:
                index = int(data_split[1])
                velocity = float(data_split[3])
                self.process_joints_state(index, velocity)
            except Exception as e:
                self.get_logger().debug(f'{e}')
                return

    def process_joints_state(self, index, velocity):
        if index == self.left_wheel_joint_index: # Left wheel joint
            self.wl = velocity
            if self.debug:
                msg = JointState()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "left_wheel"
                msg.velocity = velocity
                self.left_joint_state_pub.publish(msg)
        elif index == self.right_wheel_joint_index: # Right wheel joint
            self.wr = velocity
            if self.debug:
                msg = JointState()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = "right_wheel"
                msg.velocity = velocity
                self.right_joint_state_pub.publish(msg)

    def hb_cb(self):
        self.serial_device_write("HB")

    def forward_kinematics(self, wl, wr):
        vl = (self.wheel_diameter/2) * wl
        vr = -(self.wheel_diameter/2) * wr
        
        v = (vr + vl)/2
        w = (vr - vl)/(self.wheel_separation_multiplier 
                       * self.wheel_to_wheel_separation)
        return v, w

    def inverse_kinematics(self, v, w):
        wl = (1/(self.wheel_diameter/2)) * (v - (w * \
            self.wheel_separation_multiplier * (self.wheel_to_wheel_separation/2)))
        wr = -(1/(self.wheel_diameter/2)) * (v + (w * \
            self.wheel_separation_multiplier * (self.wheel_to_wheel_separation/2)))
        return wl, wr

    def enable_base_cb(self, req, res):
        if req.enable:
            self.enable = True
        else:
            # 1. Stop the bot
            msg = Twist()
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.cmd_vel_cb(msg)
            # 2. set enable
            self.enable = False
        res.success = True
        return res

    def start_logging_cb(self, req, res):
        if not self.logging:
            self.logging_fd = open(os.path.join(req.path_to_data_dir, "wheel_control.csv"), "w")
            self.logging_fd.write("timestamp,v,w,v_reff,w_reff\n")
            self.logging = True
            res.success = True
        else:
            res.success = False
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

    def cmd_vel_cb(self, msg):
        if not self.enable:
            return

        wl, wr = self.inverse_kinematics(msg.linear.x, msg.angular.z)
        self.serial_device_write(f"J {self.left_wheel_joint_index} {wl:.4}")
        self.serial_device_write(f"J {self.right_wheel_joint_index} {wr:.4}")
        self.v_cmd = msg.linear.x
        self.w_cmd = msg.angular.z
        self.cmd_vel_time = self.get_clock().now()
        if self.debug:
            msg = JointCmd()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "left_wheel"
            msg.cmd = wl
            self.left_joint_cmd_pub.publish(msg)
            msg.header.frame_id = "right_wheel"
            msg.cmd = wr
            self.right_joint_cmd_pub.publish(msg)

    def odom_cb(self):
        v, w = self.forward_kinematics(self.wl, self.wr)

        if self.publish_odom:
            msg = Odometry()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "odom"
            msg.child_frame_id = "base_footprint"

            if self.publish_pose_in_odom:
                if self.odom_time is None:
                    self.odom_time = self.get_clock().now()
                else:
                    dt = (self.get_clock().now() - self.odom_time).nanoseconds /10**9
                    self.odom_time = self.get_clock().now()
                    self.x += v * math.cos(self.yaw) * dt
                    self.y += v * math.sin(self.yaw) * dt
                    self.yaw += w * dt
                    msg.pose.pose.position.x = self.x
                    msg.pose.pose.position.y = self.y
                    msg.pose.pose.orientation.w = math.cos(self.yaw/2)
                    msg.pose.pose.orientation.w = math.sin(self.yaw/2)
                    for i in range(6):
                        msg.pose.covariance[7 * i] = self.pose_covariance_diagonal[i]
            
            msg.twist.twist.linear.x = v
            msg.twist.twist.angular.z = w

            for i in range(6):
                msg.twist.covariance[7 * i] = self.twist_covariance_diagonal[i]

            self.odom_pub.publish(msg)

        self.logging_lock.acquire()
        if self.logging:
            self.logging_fd.write(
                f"{self.get_clock().now().nanoseconds/1e9},{v},{w},{self.v_cmd},{self.w_cmd}\n"
            )
        self.logging_lock.release()

    def observer_cb(self):
        # 0. Wait for every thing to boot up befor observing
        dt = (self.get_clock().now() - self.start_time).nanoseconds / 10**9
        if dt < 2.0:
            return
        
        # cmd_vel
        if self.cmd_vel_timeout_enabled:
            if self.cmd_vel_time is not None:
                dt = (self.get_clock().now() - self.cmd_vel_time).nanoseconds / 10**9
                if dt >= self.cmd_vel_timeout:
                    if self.v_cmd != 0.0 or self.w_cmd != 0.0:
                        msg = Twist()
                        msg.linear.x = 0.0
                        msg.angular.z = 0.0
                        self.cmd_vel_cb(msg)
                        self.get_logger().warn('cmd_vel timedout, stopping bot')

        # 1. Observe HB
        if self.hb_time is not None:
            dt = (self.get_clock().now() - self.hb_time).nanoseconds / 1e9
            if dt >= self.hb_timeout:
                self.get_logger().warn(f'No HB received from serial_device_drive since {dt:.4} sec')
        else:
            self.get_logger().warn('No HB received from serial_device_drive yet!')


def main(args=None):
    rclpy.init(args=args)
    flo_base_diff = FloBaseDiff()
    multi_thread_executor = MultiThreadedExecutor()
    multi_thread_executor.add_node(flo_base_diff)
    try:
        rclpy.spin(flo_base_diff, executor=multi_thread_executor)
        # spin(flo_base_diff, executor=multi_thread_executor, hz=125)
    except KeyboardInterrupt:
        flo_base_diff.stop_logging_cb(StopLogging.Request(), StopLogging.Response())
        flo_base_diff.destroy_node()
        print("Killing flo_base_diff!")


if __name__ == "__main__":
    main()
