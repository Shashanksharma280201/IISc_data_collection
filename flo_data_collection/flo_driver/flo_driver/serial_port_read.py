#!/usr/bin/env python3
import sys
from threading import Thread

import rclpy
from rclpy.node import Node

import serial
from std_msgs.msg import String


class SerialPortRead(Node):
    def __init__(self):
        super().__init__('serial_port_read')
        self.device = self.declare_parameter('device', '/dev/ttyACM0').value
        self.device_name = self.declare_parameter('device_name', 'serial_port').value
        self.baudrate = self.declare_parameter('baudrate', 115200).value

        try:
            self.serial = serial.Serial(
                port=self.device,
                baudrate=self.baudrate,
                timeout=0.1
            )
        except serial.SerialException:
            self.get_logger().fatal(f"Error opening device : {self.device}")
            sys.exit(1)

        self.read_pub = self.create_publisher(
            String,
            f"{self.device_name}/out",
            10
        )

        self.read_thread = Thread(
            target=self.read_cb,
            name=f"{self.device_name}-read-thread"
        )
        self.read_thread.start()

        self.get_logger().info(f"Serial port opened on {self.device}")

    def read_cb(self):
        while rclpy.ok():
            try:
                data = self.serial.read_until(b"\n")
                data = data.decode(errors="strict")
                msg = String(data=data)
                self.read_pub.publish(msg)
            except serial.SerialException:
                self.get_logger().fatal("Device disconnected")
                return
            except Exception:
                continue

    def wait(self):
        self.read_thread.join()


def main(args=None):
    rclpy.init(args=args)
    serial_port_read = SerialPortRead()
    try:
        serial_port_read.wait()
    except KeyboardInterrupt:
        print("Killing serial_port_read!")
        serial_port_read.destroy_node()


if __name__ == "__main__":
    main()
