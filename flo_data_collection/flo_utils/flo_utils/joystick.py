#!/usr/bin/env python3

import os
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from geometry_msgs.msg import Twist

os.environ['SDL_AUDIODRIVER'] = 'dsp'
os.environ["SDL_VIDEODRIVER"] = "dummy"
import pygame

from ros2_utils import spin


class ControlMode(Enum):
    ONLY_V = 1
    ONLY_W = 2
    BOTH = 3
    DISABLE = 4


class Joystick(Node):
    def __init__(self):
        super().__init__('joystick')

        self.declare_parameter('cmd_vel_hz', 30.0)
        self.declare_parameter('v_min', 0.1)
        self.declare_parameter('v_max', 1.0)
        self.declare_parameter('w_min', 0.1)
        self.declare_parameter('w_max', 1.0)
        self.declare_parameter('zero_tolerance', 0.01)
        self.declare_parameter('enable_duration', 5.0)
        self.declare_parameter('joystick_index', 0)
        self.cmd_vel_hz = self.get_parameter(
            'cmd_vel_hz').get_parameter_value().double_value
        self.v_min = self.get_parameter(
            'v_min').get_parameter_value().double_value
        self.v_max = self.get_parameter(
            'v_max').get_parameter_value().double_value
        self.w_min = self.get_parameter(
            'w_min').get_parameter_value().double_value
        self.w_max = self.get_parameter(
            'w_max').get_parameter_value().double_value
        self.epsilon = self.get_parameter(
            'zero_tolerance').get_parameter_value().double_value
        self.enable_duration = self.get_parameter(
            'enable_duration').get_parameter_value().double_value
        self.joystick_index = self.get_parameter(
            'joystick_index').get_parameter_value().integer_value

        self.control_mode = ControlMode.DISABLE
        self.enable_start_time = None

        pygame.init()
        pygame.joystick.init()

        joystick_count = pygame.joystick.get_count()
        if joystick_count == 0:
            self.joystick_connected = False
            self.get_logger().warn("No joystick connected. Waiting ...")

        self.joystick_event_timer = self.create_timer(
            1.0/self.cmd_vel_hz, self.joystick_cb)

        self.cmd_vel_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            10
        )

    def _map(self, x, x_min, x_max, new_min, new_max, epsilon, round_to=3):
        m = (new_max - new_min) / (1 - epsilon)
        c = (new_min - epsilon * new_max) / (1 - epsilon)
        f_of_x = 0.0
        if -x_max <= x < -epsilon:
            f_of_x = float(m * x - c)
        elif -epsilon <= x <= epsilon:
            f_of_x = 0.0
        elif epsilon < x <= x_max:
            f_of_x = float(m * x + c)
        else:
            f_of_x = 0.0
        return round(f_of_x, 3)

    def joystick_cb(self):
        for event in pygame.event.get():
            event_type = event.type
            if event_type == pygame.JOYDEVICEADDED:
                if pygame.joystick.get_count() == 0:
                    return
                self.get_logger().info("Joystick connected")
                # consider only one joystick
                self.joystick = pygame.joystick.Joystick(0)
                self.joystick.init()
                self.get_logger().info(
                    f"Initialized joystick: {self.joystick.get_name()}")
                self.joystick_connected = True
                self.control_mode = ControlMode.DISABLE
            elif event_type == pygame.JOYDEVICEREMOVED:
                self.get_logger().warn("Joystick disconnected")
                self.joystick_connected = False
                self.joystick = None
                self.control_mode = ControlMode.DISABLE
            elif event.type == pygame.QUIT:
                return

        if not self.joystick_connected:
            return

        num_buttons = int(self.joystick.get_numbuttons())
        buttons = [self.joystick.get_button(i) for i in range(num_buttons)]

        if buttons[8]:
            # check if it is pressed for 1 sec
            # update counter
            if self.enable_start_time is None:
                self.enable_start_time = self.get_clock().now()

            dt = (self.get_clock().now() -
                  self.enable_start_time).nanoseconds / 10**9
            if dt >= self.enable_duration:
                self.joystick.rumble(1000, 1000, 1000)
                if self.control_mode == ControlMode.DISABLE:
                    self.control_mode = ControlMode.BOTH
                    self.get_logger().info("Enabled joystick control")
                else:
                    self.control_mode = ControlMode.DISABLE
                    self.get_logger().info("Disabled joystick control")
                self.enable_start_time = None
        else:
            # reset counter
            self.enable_start_time = None

        if self.control_mode == ControlMode.DISABLE:
            return

        if buttons[4] and not buttons[5]:
            self.control_mode = ControlMode.ONLY_W
        elif buttons[5] and not buttons[4] == 1:
            self.control_mode = ControlMode.ONLY_V
        else:
            self.control_mode = ControlMode.BOTH
        num_axes = int(self.joystick.get_numaxes())
        axes = [round(self.joystick.get_axis(i), 3)
                for i in range(0, num_axes)]
        if len(axes) != 6:
            return
        if self.joystick_index == 0:
            axes = axes[:num_axes//2]
        elif self.joystick_index == 1:
            axes = axes[-num_axes//2:]
        lin_x = -axes[1]
        ang_z = -axes[0]

        lin_x = self._map(lin_x, 0.0, 1.0, self.v_min,
                          self.v_max, self.epsilon)
        ang_z = self._map(ang_z, 0.0, 1.0, self.w_min,
                          self.w_max, self.epsilon)

        cmd_vel = Twist()
        cmd_vel.linear.x = lin_x if not self.control_mode == ControlMode.ONLY_W else 0.0
        cmd_vel.angular.z = ang_z if not self.control_mode == ControlMode.ONLY_V else 0.0

        self.cmd_vel_pub.publish(cmd_vel)


def main(args=None):
    rclpy.init(args=args)
    joystick = Joystick()
    multi_thread_executor = MultiThreadedExecutor()
    multi_thread_executor.add_node(joystick)
    try:
        spin(joystick, executor=multi_thread_executor, hz=125)
    except KeyboardInterrupt:
        joystick.destroy_node()
        pygame.quit()
        print("Killing joystick!")


if __name__ == "__main__":
    main()
