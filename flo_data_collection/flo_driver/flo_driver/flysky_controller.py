#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String
from flo_msgs.msg import FlySky
from flo_utils.ros2_utils import spin


class FlySkyController(Node):
    def __init__(self):
        super().__init__('flysky_controller')

        self.raw_input_topic = self.declare_parameter(
            'raw_input_topic', '/flysky_controller/raw_input').value
        self.sign_change = self.declare_parameter(
            'sign_change', True).value
        
        self.raw_sub = self.create_subscription(
            String,
            self.raw_input_topic,
            self.raw_input_topic_sub_cb,
            10
        )

        self.flysky_msg_pub = self.create_publisher(
            FlySky,
            "/flysky_controller/data",
            10
        )

        self.multiplier = -1 if self.sign_change else 1

    def raw_input_topic_sub_cb(self, msg):
        raw = msg.data.strip()
        if "RF" not in raw:
            return
        
        raw = raw.split(" ")
        if len(raw) != 12:
            return
        
        raw = raw[2:]
        raw = [int(x) for x in raw]

        msg = FlySky()

        msg.header.stamp = self.get_clock().now().to_msg()

        msg.right_x = FlySkyController.map(raw[0], 1000, 2000, -1, 1) * self.multiplier
        msg.right_y = FlySkyController.map(raw[1], 1000, 2000, -1, 1)
        msg.left_x = FlySkyController.map(raw[3], 1000, 2000, -1, 1) * self.multiplier
        msg.left_y = FlySkyController.map(raw[2], 1000, 2000, -1, 1)

        msg.knob_left = FlySkyController.map(raw[6], 1000, 2000, 0, 1)
        msg.knob_right = FlySkyController.map(raw[7], 1000, 2000, 0, 1)

        if raw[4] == 1000:
            msg.switch_a = FlySky.SWITCH_UP
        elif raw[4] == 2000:
            msg.switch_a = FlySky.SWITCH_DOWN
        else:
            msg.switch_a = FlySky.SWITCH_UNK

        if raw[5] == 1000:
            msg.switch_b = FlySky.SWITCH_UP
        elif raw[5] == 2000:
            msg.switch_b = FlySky.SWITCH_DOWN
        else:
            msg.switch_b = FlySky.SWITCH_UNK

        if raw[8] == 1000:
            msg.switch_c = FlySky.SWITCH_UP
        elif raw[8] == 1500:
            msg.switch_c = FlySky.SWITCH_MIDDLE
        elif raw[8] == 2000:
            msg.switch_c = FlySky.SWITCH_DOWN
        else:
            msg.switch_c = FlySky.SWITCH_UNK

        if raw[9] == 1000:
            msg.switch_d = FlySky.SWITCH_UP
        elif raw[9] == 2000:
            msg.switch_d = FlySky.SWITCH_DOWN
        else:
            msg.switch_d = FlySky.SWITCH_UNK
        
        self.flysky_msg_pub.publish(msg)

    @staticmethod
    def map(value, x_min, x_max, y_min, y_max):
        # Figure out how 'wide' each range is
        x_span = x_max - x_min
        y_span = y_max - y_min

        # Convert the left range into a 0-1 range (float)
        valueScaled = float(value - x_min) / float(x_span)

        # Convert the 0-1 range into a value in the right range.
        return y_min + (valueScaled * y_span)

def main(args=None):
    rclpy.init(args=args)
    flysky_controller = FlySkyController()
    multi_thread_executor = MultiThreadedExecutor()
    multi_thread_executor.add_node(flysky_controller)

    try:
        spin(flysky_controller, executor=multi_thread_executor, hz=125)
    except KeyboardInterrupt:
        print("Shutting down...")
        flysky_controller.destroy_node()


if __name__ == "__main__":
    main()