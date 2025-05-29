#!/usr/bin/env python3

import os
import psutil
from threading import Lock

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from flo_msgs.msg import Perf, ThermalZone, Governor, ClockFrequency
from flo_msgs.srv import StartLogging, StopLogging

from flo_utils.ros2_utils import spin


class PerfMonitor(Node):
    def __init__(self):
        super().__init__('perf_monitor')
        self.declare_parameter('hz', 0.2)
        self.hz = self.debug = self.get_parameter('hz').get_parameter_value().double_value
        self.perf_timer = self.create_timer(1.0/self.hz, self.perf_cb)

        self.perf_pub = self.create_publisher(
            Perf,
            "perf",
            10
        )

        self.logging = False
        self.logging_fd = None
        self.logging_lock = Lock()
        self.start_logging = self.create_service(
            StartLogging,
            'perf_monitor/start_logging',
            self.start_logging_cb
        )
        self.stop_logging = self.create_service(
            StopLogging,
            'perf_monitor/stop_logging',
            self.stop_logging_cb
        )

        self.get_logger().info("Started perf monitor")

    def start_logging_cb(self, req, res):
        if not self.logging:
            self.logging_fd = open(os.path.join(req.path_to_data_dir, "perf.csv"), "w")
            csv_header = \
                "timestamp,cpu_used_in_percentage,ram_used_in_mb,disk_used_in_mb,number_of_thermal_zones"
            for zone in self.get_thermal_zones():
                csv_header += f",{zone}"
            csv_header += ",number_of_cores"
            for frequencies in self.get_clock_freqs():
                csv_header += f",{frequencies}"
            csv_header += ",number_of_governors"
            for governor in self.get_governors():
                csv_header += f",{governor}"
            self.logging_fd.write(
                f"{csv_header}\n"
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
            self.logging_fd.close()
            self.logging = False
            self.logging_lock.release()
            res.success = True
        else:
            res.success = True
            res.message = "Already not logging!"
        return res
    
    def get_clock_freqs(self):
        clock_frequencies = {}
        clock_path = "/sys/bus/cpu/devices/"
        try:
            for entry in os.scandir(clock_path):
                if not entry.is_dir() or not entry.name.startswith("cpu"):
                    continue
                frequencies = os.path.join(clock_path, entry.name, "cpufreq/scaling_cur_freq")
                if not os.path.exists(frequencies):
                    continue
                with open(frequencies) as f:
                    freq = int(f.read().strip())
                    clock_frequencies[entry.name] = freq
        except Exception:
            return clock_frequencies

        return clock_frequencies
    
    def get_governors(self):
        governors = {}
        governor_path = "/sys/bus/cpu/devices/"
        try:
            for entry in os.scandir(governor_path):
                if not entry.is_dir() or not entry.name.startswith("cpu"):
                    continue
                governor = os.path.join(governor_path, entry.name, "cpufreq/scaling_governor")
                if not os.path.exists(governor):
                    continue
                with open(governor) as f:
                    gov = (f.read().strip())
                    governors[entry.name] = gov
        except Exception:
            return governors

        return governors

    def get_thermal_zones(self):
        thermal_zones = {}
        thermal_path = "/sys/class/thermal/"
        try:
            for entry in os.scandir(thermal_path):
                if not entry.is_dir() or not entry.name.startswith("thermal_zone"):
                    continue
                zone = os.path.join(thermal_path, entry.name, "temp")
                if not os.path.exists(zone):
                    continue
                with open(zone) as f:
                    temp = int(f.read().strip()) / 1000
                    thermal_zones[entry.name] = temp
        except Exception:
            return thermal_zones

        return thermal_zones
    def perf_cb(self):
        msg = Perf()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.cpu_used_in_percentage = psutil.cpu_percent(0)
        msg.ram_used_in_mb = psutil.virtual_memory().used/(1024*1024)
        msg.disk_used_in_mb = psutil.disk_usage('/').used/(1024*1024)
        thermal_zones = self.get_thermal_zones()
        clock_frequencies = self.get_clock_freqs()
        governors = self.get_governors()

        for cpu_name in clock_frequencies:
            freq = ClockFrequency()
            freq.name = cpu_name
            freq.frequency = clock_frequencies[cpu_name]
            msg.frequencies.append(freq)

        for governor_name in governors:
            gov = Governor()
            gov.name = governor_name
            gov.governor = governors[governor_name]
            msg.scaling_governors.append(gov)

        for zone_name in thermal_zones:
            thermal = ThermalZone()
            thermal.name = zone_name
            thermal.temperature = thermal_zones[zone_name]
            msg.thermals.append(thermal)

        if msg.cpu_used_in_percentage > 90.0:
            self.get_logger().warn(f'{msg.cpu_used_in_percentage}% CPU used!')

        if psutil.virtual_memory().percent > 90.0:
            self.get_logger().warn(f'{psutil.virtual_memory().percent}% RAM used!')

        if psutil.disk_usage('/').percent > 90.0:
            self.get_logger().warn(f'{psutil.disk_usage("/").percent}% Disk used!')


        self.perf_pub.publish(msg)
        self.logging_lock.acquire()
        if self.logging:
            temp_entry = ""
            freq_entry = ""
            gov_entry = ""
            for zone in msg.thermals:
                temp_entry += f"{zone.temperature},"
            temp_entry = temp_entry[:-1]

            for frequencies in msg.frequencies:
                freq_entry += f"{frequencies.frequency},"
            freq_entry = freq_entry[:-1]

            for governors in msg.scaling_governors:
                gov_entry += f"{governors.governor},"
            gov_entry = gov_entry[:-1]

            self.logging_fd.write(
                f"{self.get_clock().now().nanoseconds/1e9},"+\
                f"{msg.cpu_used_in_percentage},"+\
                f"{msg.ram_used_in_mb},"+\
                f"{msg.disk_used_in_mb},"+
                f"{len(msg.thermals)},"+
                f"{temp_entry},"+\
                f"{len(msg.frequencies)},"+\
                f"{freq_entry},"+\
                f"{len(msg.scaling_governors)},"+\
                f"{gov_entry}\n"
            )
        self.logging_lock.release()


def main(args=None):
    rclpy.init(args=args)
    perf_monitor = PerfMonitor()
    multi_thread_executor = MultiThreadedExecutor()
    multi_thread_executor.add_node(perf_monitor)
    try:
        spin(perf_monitor, executor=multi_thread_executor, hz=125)
    except KeyboardInterrupt:
        print("Killing perf_monitor!")
        perf_monitor.stop_logging_cb(StopLogging.Request(), StopLogging.Response())
        perf_monitor.destroy_node()

if __name__ == "__main__":
    main()
