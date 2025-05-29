#!/usr/bin/env python3

import os
import math
import numpy as np
from threading import Thread

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from rclpy.executors import MultiThreadedExecutor

import pyfiglet
from tf2_ros import LookupException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
import tf_transformations as tr
from geometry_msgs.msg import PoseStamped, Transform
from nav_msgs.msg import Path
from flo_msgs.action import WaypointMission
from flo_msgs.msg import Waypoint

from ros2_utils import spin


def print_characters(count=1, character="-"):
    print(f"{character}"*count)


def transform_to_pq(msg: Transform):
    p = np.array([msg.translation.x, msg.translation.y, msg.translation.z])
    q = np.array([msg.rotation.x, msg.rotation.y,
                  msg.rotation.z, msg.rotation.w])
    return p, q


def tf_msg_to_tf_matrix(msg):
    p, q = transform_to_pq(msg)
    norm = np.linalg.norm(q)
    if np.abs(norm - 1.0) > 1e-3:
        raise ValueError(
            f"Received un-normalized quaternion (q = {q} ||q|| = {np.linalg.norm(q)})")
    elif np.abs(norm - 1.0) > 1e-6:
        q = q / norm
    g = tr.quaternion_matrix(q)
    g[0:3, -1] = p
    return g


class QuitProgram(Exception):
    pass


class CommandParseError(Exception):
    pass


class MissionState:
    def __init__(self, node: Node, waypoints=[], repeat=1):
        self.waypoints = waypoints
        self.repeat = repeat
        self.start_point = None
        self.mission_execute_frame = None
        self.final_yaw = None
        self.node = node
        self.path = None

        self._action_client = ActionClient(
            self.node,
            WaypointMission,
            '/waypoint_mission'
        )

        self.in_progress = True

    def _goal_response_cb(self, future):
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.node.get_logger().warn('mission action goal rejected!')
            return

        self.get_result_future = self.goal_handle.get_result_async()
        self.get_result_future.add_done_callback(self._get_result_cb)

    def _get_result_cb(self, future):
        result = future.result().result
        if not result.success:
            self.node.get_logger().warn(
                f"Mission unsuccessful : {result.message}")
        else:
            self.node.get_logger().info("Completed mission successfully")
        self.in_progress = False

    def _feedback_cb(self, feedback_msg):
        self.feedback = feedback_msg.feedback

    def display_feedback(self):
        if self.feedback is None:
            if self.in_progress:
                print("No Feedack received")
            return
        print("Mission Status:")
        print(f"Points done: {self.feedback.number_of_waypoints_done}")
        print(f"Total Points: {len(self.waypoints)}")

    def execute(self):
        Thread(target=self._run).start()

    def _run(self):
        self.in_progress = True
        goal = WaypointMission.Goal()
        goal.waypoints = self.waypoints
        goal.frame = self.mission_execute_frame

        self._action_client.wait_for_server()

        self.goal_future = self._action_client.send_goal_async(
            goal, feedback_callback=self._feedback_cb)
        self.goal_future.add_done_callback(self._goal_response_cb)

    def abort(self):
        if not self.in_progress:
            return

        self.goal_handle.cancel_goal_async()

    def return_to_start(self):
        pass


class WaypointMissionClient(Node):
    def __init__(self):
        super().__init__('waypoint_mission_client')

        self.display_intro()
        self.cli_thread = Thread(target=self.cli_loop)
        self.cli_thread.start()

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self.path_pub = self.create_publisher(
            Path,
            "/planned_path",
            10
        )

        self.mission = None

    def display_intro(self):
        print(pyfiglet.figlet_format("Flo Mobility"))
        print("Mission client")
        print_characters(count=32, character="*")
        self.display_help_text()

    def display_help_text(self):
        print(r"""Available Commands:
m - input mission
a - abort mission
s - mission status
h - help text
c - clear screen
q - quit program

------------------------------------------------
Turtle bot based command syntax
              
Directions (Relative to bot):
------------------------------------------------
F - Forward          F
B - Back             ↑
L - Left        L ← Bot → R
R - Right            ↓
                     B

Command Syntax: <Direction> <distance in meters>
------------------------------------------------
Sample usage:
Basic → F 1.1,R 4.3;
With Repeat → F 1.1,R 4.3*10; this repeats 10 times
""", end="")

    def is_valid_direction(self, cmd: str):
        try:
            if len(cmd) < 2:
                return False, []

            if cmd[0] not in ['F', 'B', 'L', 'R']:
                return False, []

            direction_value = float(cmd[1:].rstrip())
            return True, [cmd[0], direction_value]
        except Exception as e:
            raise CommandParseError(e)

    def parse_cmd_string(self, cmd: str, target_frame: str):
        if cmd.lower() == 'q':
            raise QuitProgram()

        if cmd[-1] != ';':
            raise CommandParseError("Expected ; to terminate command.")

        mission = MissionState(self)

        cmd = cmd[:-1]
        repeat_split = cmd.split("*")
        if len(repeat_split) != 1:
            mission.repeat = int(repeat_split[1])

        T_bf_mission = None
        try:
            T_bf_mission = self._tf_buffer.lookup_transform(
                target_frame, "base_footprint", Time())
        except LookupException:
            raise CommandParseError(
                f"Error in looking up T_bf_{target_frame}")

        if T_bf_mission is None:
            raise CommandParseError(
                f"Error in looking up T_bf_{target_frame}")

        T_bf_mission = tf_msg_to_tf_matrix(T_bf_mission.transform)

        start_point = Waypoint()
        start_point.x = 0.0
        start_point.y = 0.0

        P_bf = np.array([start_point.x, start_point.y, 0.0, 1.0])

        P_mission = T_bf_mission @ P_bf.T

        start_point_mission_frame = Waypoint()
        start_point_mission_frame.x = P_mission[0]
        start_point_mission_frame.y = P_mission[1]
        mission.start_point = start_point_mission_frame  # in mission frame

        waypoints = []
        waypoints_in_mission = []
        direction_commands = repeat_split[0].split(",")
        for idx, direction_cmd in enumerate(direction_commands):
            success, parsed_direction = self.is_valid_direction(
                direction_cmd.rstrip())
            if not success:
                raise CommandParseError(f"Invalid direction : {direction_cmd}")

            target = Waypoint()
            direction = parsed_direction[0]
            value = parsed_direction[1]
            previous_waypoint = start_point if idx == 0 else waypoints[idx - 1]

            if direction == 'F':
                target.x = previous_waypoint.x + value
                target.y = previous_waypoint.y
            elif direction == 'B':
                target.x = previous_waypoint.x - value
                target.y = previous_waypoint.y
            elif direction == 'L':
                target.x = previous_waypoint.x
                target.y = previous_waypoint.y + value
            elif direction == 'R':
                target.x = previous_waypoint.x
                target.y = previous_waypoint.y - value

            P_bf = np.array([target.x, target.y, 0.0, 1.0])

            P_mission = T_bf_mission @ P_bf.T

            target.x = P_bf[0]
            target.y = P_bf[1]

            target_in_mission_frame = Waypoint()
            target_in_mission_frame.x = P_mission[0]
            target_in_mission_frame.y = P_mission[1]

            waypoints.append(target)
            waypoints_in_mission.append(target_in_mission_frame)

        if mission.repeat > 1:
            repeat_list = []
            repeat_list.append(mission.start_point)
            repeat_list += waypoints_in_mission

            waypoints_in_mission += repeat_list * (mission.repeat - 1)

        mission.waypoints = waypoints_in_mission

        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = target_frame
        waypoints = [mission.start_point] + mission.waypoints
        for idx, waypoint in enumerate(waypoints):
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.header.frame_id = target_frame
            pose_stamped.pose.position.x = waypoint.x
            pose_stamped.pose.position.y = waypoint.y
            if idx != len(waypoints) - 1:
                heading = math.atan2(
                    waypoints[idx+1].y - waypoint.y, waypoints[idx+1].x - waypoint.x)
                if idx == len(waypoints) - 2:
                    mission.final_yaw = heading
            else:
                heading = mission.final_yaw
            pose_stamped.pose.orientation.w = math.cos(heading/2)
            pose_stamped.pose.orientation.z = math.sin(heading/2)
            path.poses.append(pose_stamped)
            mission.path = path
        return mission

    def publish_path(self, mission: MissionState):
        self.path_pub.publish(mission.path)

    def cli_loop(self):
        while rclpy.ok():
            try:
                user_input_raw = input("> ").rstrip()
                # self.get_logger().info(f"You inputted : {user_input}") # debug
                user_input = user_input_raw.lower()
                if user_input == 'q':
                    raise QuitProgram()
                elif user_input == 'h':
                    self.display_help_text()
                elif user_input == 'c':
                    os.system("clear")
                elif user_input == 'm':
                    if self.mission is not None and self.mission.in_progress:
                        print("Mission currently running, abort to give new mission")
                        continue
                    target_frame = input("Mission execution frame: ").rstrip()
                    cmd_string = input("Directions in bf frame: ").rstrip()
                    self.mission = self.parse_cmd_string(
                        cmd_string, target_frame)
                    self.mission.mission_execute_frame = target_frame
                    if self.mission is None:
                        continue
                    self.publish_path(self.mission)
                    self.mission.execute()
                elif user_input == 'a':
                    self.mission.abort()
                    self.mission = None
                elif user_input == 's':
                    if self.mission is None:
                        continue
                    self.mission.display_feedback()
                elif user_input == 't':
                    parent_frame = input("Parent frame: ").rstrip()
                    child_frame = input("Child frame: ").rstrip()
                    try:
                        T_child_parent = self._tf_buffer.lookup_transform(
                            parent_frame, child_frame, Time()
                        )
                        print(T_child_parent)
                        T_child_parent = tf_msg_to_tf_matrix(
                            T_child_parent.transform)
                        print(T_child_parent)
                    except LookupException:
                        self.get_logger()\
                            .warn(f"Error in looking up T_{child_frame}_{parent_frame}")

                else:
                    print(f"Unknown command :{user_input_raw}")
            except CommandParseError as e:
                self.get_logger().error(f"CommandParseError : {e}")
            except QuitProgram:
                self.shutdown_mission_client()
            except KeyboardInterrupt:
                return

    def shutdown_mission_client(self):
        self.get_logger().info("Shutting down waypoint mission client ...")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    waypoint_mission_client = WaypointMissionClient()
    multi_thread_executor = MultiThreadedExecutor()
    multi_thread_executor.add_node(waypoint_mission_client)
    try:
        spin(waypoint_mission_client, executor=multi_thread_executor, hz=25)
    except KeyboardInterrupt:
        print("Shutting down waypoint mission client ... Press Ctrl+C again")
    finally:
        waypoint_mission_client.destroy_node()


if __name__ == '__main__':
    main()
