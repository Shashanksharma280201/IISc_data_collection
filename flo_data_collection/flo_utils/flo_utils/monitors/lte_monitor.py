#!/usr/bin/env python3

import os
import time
from threading import Lock

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup

from std_msgs.msg import String
from sensor_msgs.msg import Temperature
from flo_msgs.msg import LteInfo, Voltage
from flo_msgs.srv import StartLogging, StopLogging, GetSimId

from flo_utils.ros2_utils import spin, Rate


def safe_int(field, default=0):
    try:
        return int(field)
    except TypeError:
        return default
    except ValueError:
        return default


class LteMonitor(Node):
    def __init__(self):
        super().__init__('lte_monitor')
        self.declare_parameter('hz', 0.2)
        self.hz = self.get_parameter(
            'hz').get_parameter_value().double_value

        self.lte_publisher = self.create_publisher(
            LteInfo,
            "/lte_monitor/stats/signal",
            10
        )

        self.lte_modem_temp_pub = self.create_publisher(
            Temperature,
            "/lte_monitor/stats/temperature",
            10
        )

        self.lte_modem_volt_pub = self.create_publisher(
            Voltage,
            "/lte_monitor/stats/voltage",
            10
        )

        self.at_command_pub = self.create_publisher(
            String,
            "/lte_monitor/in",
            10
        )

        self.lte_stats_raw_data_sub = self.create_subscription(
            String,
            "/lte_monitor/out",
            self.lte_raw_data_cb,
            10,
            callback_group=ReentrantCallbackGroup()
        )

        self.sim_id = None
        self.logging = False
        self.logging_signal_quality_fd = None
        self.logging_temperature_fd = None
        self.logging_voltage_fd = None
        self.logging_lock = Lock()
        self.at_lock = Lock()
        self.start_logging = self.create_service(
            StartLogging,
            'lte_monitor/start_logging',
            self.start_logging_cb,
            callback_group=MutuallyExclusiveCallbackGroup()
        )
        self.stop_logging = self.create_service(
            StopLogging,
            'lte_monitor/stop_logging',
            self.stop_logging_cb,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        self.get_sim_id = self.create_service(
            GetSimId, 
            '/lte_monitor/get_sim_id', 
            self.get_sim_id_cb,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        self.lte_monitor_timer = self.create_timer(
            1.0/self.hz, 
            self.lte_monitor_cb,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        self.get_logger().info("Started lte_monitor")
        
    def get_sim_id_cb(self, request, response):
        rate = Rate(hz=1)
        attempt = 0

        self.sim_id = None

        self.at_lock.acquire()
        while self.sim_id is None:
            if attempt == 5:
                self.get_logger().error('Error in getting sim id. \
                                        Service failed to respond in time')
                response.success = False
                response.sim_id = ""
                response.message = "Operation timed out"
                self.at_lock.release()
                return response
            msg = String(data="AT+CCID")
            self.at_command_pub.publish(msg)
            attempt += 1
            rate.sleep()
        self.at_lock.release()

        response.success = True
        response.sim_id = self.sim_id
        return response

    def start_logging_cb(self, req, res):
        if not self.logging:
            self.logging_signal_quality_fd = open(os.path.join(
                req.path_to_data_dir, "lte_signal_quality.csv"), "w")
            self.logging_signal_quality_fd.write(
                "timestamp,system_mode,operation_mode," +
                "mcc,mnc,tac,scell_id,pcell_id,frequency_band," +
                "earfcn,dlbw,ulbw,rsrq,rsrp,rssi,rssnr\n"
            )
            self.logging_temperature_fd = open(os.path.join(
                req.path_to_data_dir, "lte_modem_temperature.csv"), "w")
            self.logging_temperature_fd.write(
                "timestamp,temperature\n"
            )
            self.logging_voltage_fd = open(os.path.join(
                req.path_to_data_dir, "lte_modem_voltage.csv"), "w")
            self.logging_voltage_fd.write(
                "timestamp,voltage\n"
            )
            self.logging = True
            res.success = True
        else:
            res.success = False
            res.message = "Already logging!"
        return res

    def stop_logging_cb(self, req, res):
        if self.logging:
            self.logging_lock.acquire()
            self.logging_signal_quality_fd.close()
            self.logging_temperature_fd.close()
            self.logging_voltage_fd.close()
            self.logging = False
            self.logging_lock.release()
            res.success = True
        else:
            res.success = True
            res.message = "Already not logging!"
        return res

    def lte_raw_data_cb(self, msg: String):
        response = msg.data

        if "+CPSI" in response:
            self.handle_lte_stats(msg)
        if "+CCID" in response:
            self.handle_sim_id(msg)
        if "+CPMUTEMP" in response:
            self.handle_modem_temp(msg)
        if "+CBC" in response:
            self.handle_modem_volt(msg)

    def handle_modem_volt(self, msg):
        response = msg.data
        volt = response.strip("+CBC: ").rstrip("V\r\r")
        if volt == '':
            return
        
        try:
            volt = float(volt)
        except ValueError:
            return

        stamp = self.get_clock().now()

        volt_msg = Voltage()
        volt_msg.header.stamp = stamp.to_msg()
        volt_msg.header.frame_id = "SIM7600 4g_modem - Qualcomm"

        volt_msg.voltage = volt
        self.lte_modem_volt_pub.publish(volt_msg)

        self.logging_lock.acquire()
        if self.logging:
            self.logging_voltage_fd.write(
                f"{stamp.nanoseconds/1e9}," +
                f"{volt_msg.voltage}\n"
            )
        self.logging_lock.release()

    def handle_modem_temp(self, msg):
        response = msg.data
        temp = response.strip("+CPMUTEMP: ")
        if temp == '':
            return
        
        temp = float(temp)

        stamp = self.get_clock().now()

        temp_msg = Temperature()
        temp_msg.header.stamp = stamp.to_msg()
        temp_msg.header.frame_id = "SIM7600 4g_modem - Qualcomm"

        temp_msg.temperature = temp
        self.lte_modem_temp_pub.publish(temp_msg)

        self.logging_lock.acquire()
        if self.logging:
            self.logging_temperature_fd.write(
                f"{stamp.nanoseconds/1e9}," +
                f"{temp_msg.temperature}\n"
            )
        self.logging_lock.release()

    def handle_sim_id(self, msg):
        reponse = msg.data
        self.sim_id = reponse.strip("+CCID: ")

    def handle_lte_stats(self, msg):
        response = msg.data
        info = response.strip("+CPSI: ").split(",")

        try:
            msg = LteInfo()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "SIM7600 4g_modem - Qualcomm"
            msg.system_mode = info[0]
            msg.operation_mode = info[1]

            mcc_mnc = info[2].split("-")
            msg.mcc = mcc_mnc[0]
            msg.mnc = mcc_mnc[1]

            msg.tac = info[3]
            msg.scell_id = safe_int(info[4])
            msg.pcell_id = safe_int(info[5])
            msg.frequency_band = info[6]
            msg.earfcn = safe_int(info[7])
            msg.dlbw = safe_int(info[8])
            msg.ulbw = safe_int(info[9])
            msg.rsrq = safe_int(info[10])
            msg.rsrp = safe_int(info[11])
            msg.rssi = safe_int(info[12])
            msg.rssnr = safe_int(info[13])

            self.lte_publisher.publish(msg)

            self.logging_lock.acquire()
            if self.logging:
                self.logging_signal_quality_fd.write(
                    f"{self.get_clock().now().nanoseconds/1e9}," +
                    f"{msg.system_mode}," +
                    f"{msg.operation_mode}," +
                    f"{msg.mcc}," +
                    f"{msg.mnc}," +
                    f"{msg.tac}," +
                    f"{msg.scell_id}," +
                    f"{msg.pcell_id}," +
                    f"{msg.frequency_band}," +
                    f"{msg.earfcn}," +
                    f"{msg.dlbw}," +
                    f"{msg.ulbw}," +
                    f"{msg.rsrq}," +
                    f"{msg.rsrp}," +
                    f"{msg.rssi}," +
                    f"{msg.rssnr}\n"
                )
            self.logging_lock.release()
        except Exception:
            self.get_logger().warn(f"Invalid data received: {response}")

    def lte_monitor_cb(self):
        self.at_lock.acquire()
        self.at_command_pub.publish(String(data="ATE0"))
        self.at_command_pub.publish(String(data="AT+CPSI?"))
        self.at_command_pub.publish(String(data="AT+CPMUTEMP"))
        time.sleep(0.5)
        self.at_command_pub.publish(String(data="AT+CBC"))
        self.at_lock.release()


def main(args=None):
    rclpy.init(args=args)
    lte_monitor = LteMonitor()
    multi_thread_executor = MultiThreadedExecutor()
    multi_thread_executor.add_node(lte_monitor)
    try:
        spin(lte_monitor, executor=multi_thread_executor, hz=125)
    except KeyboardInterrupt:
        print("Killing lte monitor ..!")
        lte_monitor.stop_logging_cb(
            StopLogging.Request(), StopLogging.Response())
        lte_monitor.destroy_node()


if __name__ == "__main__":
    main()
