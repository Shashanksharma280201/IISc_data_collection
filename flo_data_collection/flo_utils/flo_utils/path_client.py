#!/usr/bin/env python3

import os
import csv
from threading import Thread

import pyfiglet

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor

from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from flo_msgs.msg import Path2D, Point2D
from flo_msgs.action import Path as PathAction
from ros2_utils import spin


def safe_float(field, default="NaN"):
    try:
        return float(field)
    except TypeError:
        return float(default)
    except ValueError:
        return float(default)


def print_characters(count=1, character="-"):
    print(f"{character}"*count)


class QuitProgram(Exception):
    pass


class CommandParseError(Exception):
    pass


class PathMissionState:
    def __init__(self, node: Node, path):
        self.node = node
        self.path = path
        self.execute_frame = None
        self.goal_handle = None

        self._action_client = ActionClient(
            node=self.node,
            action_type=PathAction,
            action_name="/path"
        )

    def _goal_response_cb(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.node.get_logger().warn('mission action goal rejected!')
            return

        self.get_result_future = self.goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self._goal_result_cb)

    def _goal_result_cb(self, future):
        result = future.result().result
        if not result.success:
            self.node.get_logger().warn(
                f'Mission unsuccessful ({result.error_code}): {result.message}')
        else:
            self.node.get_logger().info('Completed path mission successfully')

    def _feedback_cb(self, feedback_msg):
        self.feedback = feedback_msg.feedback

    def execute(self):
        Thread(target=self._run).start()

    def _run(self):
        goal = PathAction.Goal()
        goal.header.stamp = self.node.get_clock().now().to_msg()
        goal.header.frame_id = self.execute_frame
        goal.path = self.path
        goal.path_to_logdir = os.path.join(os.path.expanduser("~"), "pure_pursuit_tracker")

        if not os.path.isdir(goal.path_to_logdir):
            os.makedirs(goal.path_to_logdir)

        self._action_client.wait_for_server()

        self.goal_future = self._action_client.send_goal_async(
            goal, feedback_callback=self._feedback_cb)
        self.goal_future.add_done_callback(self._goal_response_cb)

    def abort(self):
        self.goal_handle.cancel_goal_async()


class PathClient(Node):
    def __init__(self):
        super().__init__('path_client')

        self.display_intro()
        self.cli_thread = Thread(target=self.cli_loop)
        self.cli_thread.start()

        self.path = None
        self.frame = None
        self.path_pub = self.create_publisher(
            Path,
            "/loaded_path",
            10
        )

    def display_intro(self):
        print(pyfiglet.figlet_format("Flo Mobility"))
        print("Path client")
        print_characters(count=32, character="*")
        self.display_help_text()

    def display_help_text(self):
        help_text = "Available Commands:\n" +\
            "l - load path\n" +\
            "v - visualize path\n" +\
            "a - abort path mission\n" +\
            "e - execute path mission\n" +\
            "h - help text\n" +\
            "c - clear screen\n" +\
            "q - quit program\n"
        print(help_text)

    def load_path_from_file(self, file_path):
        with open(file_path) as f:
            reader = csv.reader(f)
            path = Path2D()
            for idx, row in enumerate(reader):
                if idx == 0:
                    continue
                point2d = Point2D()
                point2d.x = safe_float(row[1])
                point2d.y = safe_float(row[2])
                path.points.append(point2d)
            return path

    def viz_path(self, frame):
        path_msg = Path()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = frame

        for point in self.path.points:
            pose = PoseStamped()
            pose.header.stamp = self.get_clock().now().to_msg()
            pose.header.frame_id = frame
            pose.pose.position.x = point.x
            pose.pose.position.y = point.y
            path_msg.poses.append(pose)

        self.path_pub.publish(path_msg)

    def cli_loop(self):
        while rclpy.ok():
            try:
                user_input_raw = input("> ").rstrip()
                user_input = user_input_raw.lower()
                # self.get_logger().info(f"You inputted : {user_input}") # debug
                if user_input == 'q':
                    raise QuitProgram()
                elif user_input == 'c':
                    os.system("clear")
                elif user_input == 'h':
                    self.display_help_text()
                elif user_input == 'l':
                    path_to_file = input("Path file : ").strip()
                    self.frame = input("Frame : ").strip()
                    path_to_file = os.path.abspath(path_to_file)
                    if not os.path.isfile(path_to_file):
                        self.get_logger().warn(f"{path_to_file} doesn't exist")
                        continue

                    self.path = self.load_path_from_file(path_to_file)
                    if self.path is None:
                        self.get_logger().warn("Failed to load path")
                        continue

                    self.path_mission_state = PathMissionState(self, self.path)
                    self.path_mission_state.execute_frame = self.frame
                    self.get_logger().info(
                        f"Successfully loaded path with {len(self.path.points)} points")
                elif user_input == 'a':
                    if self.path_mission_state is None:
                        continue
                    self.path_mission_state.abort()
                elif user_input == 'v':
                    if self.path is None:
                        self.get_logger().warn("Path needs to be loaded to visualize")
                        continue
                    self.viz_path(self.frame)
                elif user_input == 'e':
                    if self.path_mission_state is None:
                        continue
                    self.path_mission_state.execute()
                else:
                    print(f"Unknown command :{user_input_raw}")
            except CommandParseError as e:
                self.get_logger().error(f"CommandParseError : {e}")
            except QuitProgram:
                self.get_logger().info("Shutting down waypoint mission client ...")
                rclpy.shutdown()
                return
            except KeyboardInterrupt:
                return


def main(args=None):
    rclpy.init(args=args)
    path_client = PathClient()
    multi_thread_executor = MultiThreadedExecutor()
    multi_thread_executor.add_node(path_client)
    try:
        spin(path_client, executor=multi_thread_executor, hz=25)
    except KeyboardInterrupt:
        print("Shutting down path client")
        path_client.destroy_node()


if __name__ == '__main__':
    main()
