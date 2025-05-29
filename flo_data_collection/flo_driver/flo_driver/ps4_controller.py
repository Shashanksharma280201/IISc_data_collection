#!/usr/bin/env python3

import os
from enum import Enum
from threading import Thread

import rclpy
from rclpy.node import Node
from rclpy.clock import Duration
from rclpy.executors import MultiThreadedExecutor

os.environ['PYGAME_HIDE_SUPPORT_PROMPT'] = "hide"
os.environ['SDL_AUDIODRIVER'] = 'dsp'
os.environ["SDL_VIDEODRIVER"] = "dummy"
import pygame
from ds4drv.exceptions import DeviceError
from ds4drv.eventloop import EventLoop
from flo_msgs.msg import Gamepad, Ps4ControllerStatus
from flo_msgs.srv import SetPs4ControllerLed, VibratePs4Controller, SetPs4ControllerLedFlash
from flo_utils.ros2_utils import spin
from ps4_controller_utils import HidrawBackend


class Logger:
    def __init__(self, module):
        self.module = module

    @staticmethod
    def new_module(module):
        return Logger(module)

    def error(self, msg, *args, **kwargs):
        msg = self._format_msg_(msg, *args, **kwargs)
        rclpy.logging.get_logger("ds4drv").error(msg)

    def warning(self, msg, *args, **kwargs):
        msg = self._format_msg_(msg, *args, **kwargs)
        rclpy.logging.get_logger("ds4drv").warning(msg)

    def info(self, msg, *args, **kwargs):
        msg = self._format_msg_(msg, *args, **kwargs)
        rclpy.logging.get_logger("ds4drv").info(msg)

    def debug(self, msg, *args, **kwargs):
        msg = self._format_msg_(msg, *args, **kwargs)
        rclpy.logging.get_logger("ds4drv").debug(msg)

    def _format_msg_(self, msg, *args, **kwargs):
        msg = msg.format(*args, **kwargs)
        return f"[{self.module}]: {msg}"


class ControllerThread(Thread):
    # Reference: https://www.psdevwiki.com/ps4/DualShock_4#Specifications
    MAX_VOLTAGE = 3.65
    # Reference: https://www.psdevwiki.com/ps4/DS4-USB#cite_note-2
    TOUCHPAD_MAX_X = 1919
    TOUCHPAD_MAX_Y = 942
    BATTERY_FULL_CHARGING = 11
    BATTERY_MAX = 8

    def __init__(self):
        super().__init__(target=self.run)
        self.device = None
        self.loop = EventLoop()

        self._led = (0, 0, 1)
        self._led_flash = (0, 0)

    def fire_event(self, event, *args):
        self.loop.fire_event(event, *args)

    def setup_device(self, device):
        self.device = device
        self.fire_event("device-setup", device)
        self.loop.add_watcher(device.report_fd, self.read_report)

    def cleanup_device(self):
        if self.device is None:
            return

        self.fire_event("device-cleanup")
        self.loop.remove_watcher(self.device.report_fd)
        self.device.close()
        self.device = None

    def read_report(self):
        report = self.device.read_report()

        if not report:
            if report is False:
                return

            self.cleanup_device()
            return

        self.fire_event("device-report", report)

    def run(self):
        self.loop.run()

    def exit(self):
        self.loop.stop()
        if self.device is not None:
            self.cleanup_device()
        if self.is_alive():
            self.join()

    def control(
        self,
        led_red=None,
        led_green=None,
        led_blue=None,
        rumble_small=None,
        rumble_big=None,
        flash_on=None,
        flash_off=None,
    ):
        """
        Similar to DS4Device.control but with None as default values
        :param led_red:
        :param led_green:
        :param led_blue:
        :param rumble_small:
        :param rumble_big:
        :param flash_on:
        :param flash_off:
        :raises OSError: can be raised 
            if we try to call this method when the 
            controller unexpectedly disconnects
        :return
        """
        self._led = (
            self._led[0] if led_red is None else led_red,
            self._led[1] if led_green is None else led_green,
            self._led[2] if led_blue is None else led_blue,
        )
        self._led_flash = (
            self._led_flash[0] if flash_on is None else flash_on,
            self._led_flash[1] if flash_off is None else flash_off,
        )
        # Once to change LED flashing state
        self._control()
        # Then to actually execute the control
        self._control(
            small_rumble=rumble_small if rumble_small is not None else 0,
            big_rumble=rumble_big if rumble_big is not None else 0,
        )

    def _control(self, **kwargs):
        if self.device is not None:
            self.device.control(
                led_red=self._led[0],
                led_green=self._led[1],
                led_blue=self._led[2],
                flash_led1=self._led_flash[0],
                flash_led2=self._led_flash[1],
                **kwargs
            )


class Ps4ControllerState(Enum):
    CONNECTED = 0
    LOW_BATTERY = 1
    DISCONNECTED = 2


class Ps4Controller(Node):
    def __init__(self):
        super().__init__('ps4_controller')

        pygame.init()
        pygame.joystick.init()

        self.declare_parameter('device', "")
        self.declare_parameter('backend', 'hidraw')
        self.declare_parameter('epsilon', 0.1)
        self.declare_parameter('joystick_hz', 15.0)
        self.declare_parameter('low_battery_threshold', 15.0)

        self.device_addr = self.get_parameter(
            'device').get_parameter_value().string_value
        self.backend_type = self.get_parameter(
            'backend').get_parameter_value().string_value
        self.epsilon = self.get_parameter(
            'epsilon').get_parameter_value().double_value
        self.joystick_hz = self.get_parameter(
            'joystick_hz').get_parameter_value().double_value
        self.low_battery_threshold = self.get_parameter(
            'low_battery_threshold').get_parameter_value().double_value

        self.controller = None

        self.set_led_service = self.create_service(
            SetPs4ControllerLed,
            "/ps4_controller/set_led",
            callback=self.set_led_cb
        )

        self.vibrate_controller_service = self.create_service(
            VibratePs4Controller,
            "/ps4_controller/vibrate",
            callback=self.vibrate_controller_cb
        )

        self.set_led_flash_Service = self.create_service(
            SetPs4ControllerLedFlash,
            "/ps4_controller/set_led/flash",
            callback=self.set_lef_flash_cb
        )

        self.gamepad_pub = self.create_publisher(
            Gamepad,
            "/ps4_controller/gamepad",
            10
        )

        self.status_pub = self.create_publisher(
            Ps4ControllerStatus,
            "/ps4_controller/status",
            10
        )

        self._last_status_publish_time = None

        self.state = Ps4ControllerState.DISCONNECTED

        # battery related
        self.battery_percentage = 0.0
        # vibration
        self.rumble_timer = None

        self.create_timer(1.0/5, callback=self.joystick_observer)
        self.create_timer(1.0, callback=self.status_pub_cb)
    
    def joystick_observer(self):
        try:
            for event in pygame.event.get():
                event_type = event.type
                if event_type == pygame.JOYDEVICEADDED:
                    if pygame.joystick.get_count() == 0:
                        return
                    
                    if not self.state == Ps4ControllerState.DISCONNECTED:
                        return

                    for idx in range(pygame.joystick.get_count()):
                        joystick = pygame.joystick.Joystick(idx)
                        if "ps4" in joystick.get_name().lower():
                            self.start()
                            return
                elif event_type == pygame.JOYDEVICEREMOVED:
                    if self.state == Ps4ControllerState.DISCONNECTED:
                        return
                    
                    self.get_logger().warn("PS4 controller disconnected")
                    self.state = Ps4ControllerState.DISCONNECTED
                    self.stop()
                elif event.type == pygame.QUIT:
                    return
        except Exception:
            return
            
    def status_pub_cb(self):
        msg = Ps4ControllerStatus()
        msg.connected = \
            not self.state == Ps4ControllerState.DISCONNECTED
        msg.battery_percent = self.battery_percentage
        
        self.status_pub.publish(msg)

    def cb_report(self, report):
        now = self.get_clock().now()
        if self.joystick_hz > 0 and self._last_status_publish_time is not None:
            dt = (now - self._last_status_publish_time).nanoseconds / 1e9
            if dt <= (1 / self.joystick_hz):
                return
            
        if not report:
            if report is False:
                return
            
            self.state = Ps4ControllerState.DISCONNECTED
            return

        msg = Gamepad()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = self.controller.device.device_addr

        msg.right.x = -Ps4Controller._normalize_axis_(
            report.right_analog_x, self.epsilon
        )
        msg.right.y = -Ps4Controller._normalize_axis_(
            report.right_analog_y, self.epsilon
        )

        msg.left.x = -Ps4Controller._normalize_axis_(
            report.left_analog_x, self.epsilon
        )
        msg.left.y = -Ps4Controller._normalize_axis_(
            report.left_analog_y, self.epsilon
        )

        msg.r2 = report.r2_analog / 255.0

        msg.l2 = report.l2_analog / 255.0

        msg.button_dpad_up = report.dpad_up
        msg.button_dpad_down = report.dpad_down
        msg.button_dpad_left = report.dpad_left
        msg.button_dpad_right = report.dpad_right
        msg.button_circle = report.button_circle
        msg.button_cross = report.button_cross
        msg.button_square = report.button_square
        msg.button_triangle = report.button_triangle
        msg.button_l1 = report.button_l1
        msg.button_l2 = report.button_l2
        msg.button_l3 = report.button_l3
        msg.button_r1 = report.button_r1
        msg.button_r2 = report.button_r2
        msg.button_r3 = report.button_r3
        msg.button_share = report.button_share
        msg.button_options = report.button_options
        msg.button_trackpad = report.button_trackpad
        msg.button_ps = report.button_ps

        self.gamepad_pub.publish(msg)
        self._last_status_publish_time = now

        # update battery
        if report.battery == ControllerThread.BATTERY_FULL_CHARGING:
            self.battery_percentage = 100.0
        else:
            self.battery_percentage = 100.0 * float(
                report.battery) / ControllerThread.BATTERY_MAX
            
        if self.battery_percentage <= self.low_battery_threshold:
            if self.state == Ps4ControllerState.CONNECTED:
                self.controller.control(
                    led_red=255,
                    led_blue=0,
                    led_green=0,
                    flash_on=40,
                    flash_off=75
                )
                self.state = Ps4ControllerState.LOW_BATTERY
                self.get_logger().warn("Low battery! Charge controller.")
        else:
            if self.state == Ps4ControllerState.LOW_BATTERY:
                self.controller.control(
                    flash_on=0,
                    flash_off=0
                )
            self.state = Ps4ControllerState.CONNECTED

    def set_led_cb(self,
                   req: SetPs4ControllerLed.Request,
                   res):
        if self.state == Ps4ControllerState.DISCONNECTED:
            res.success = False
            res.message = "PS4 controller not connected"
            return res

        if self.state == Ps4ControllerState.LOW_BATTERY:
            res.success = False
            res.message = "Low battery"
            return res

        try:
            self.controller.control(
                led_red=req.red,
                led_green=req.green,
                led_blue=req.blue
            )
            res.success = True
        except OSError:
            res.success = False
            res.message = "Error setting led"

        return res

    def set_lef_flash_cb(self,
                         req: SetPs4ControllerLedFlash.Request, res):

        if self.state == Ps4ControllerState.DISCONNECTED:
            res.success = False
            res.message = "PS4 controller not connected"
            return res

        if self.state == Ps4ControllerState.LOW_BATTERY:
            res.success = False
            res.message = "Low battery"
            return res

        try:
            if req.flash_speed == SetPs4ControllerLedFlash.Request.FAST:
                self.controller.control(
                    flash_on=10,
                    flash_off=15
                )
            elif req.flash_speed == SetPs4ControllerLedFlash.Request.SLOW:
                self.controller.control(
                    flash_on=100,
                    flash_off=150
                )
            elif req.flash_speed == SetPs4ControllerLedFlash.Request.NORMAL:
                self.controller.control(
                    flash_on=40,
                    flash_off=75
                )
            else:
                self.controller.control(
                    flash_on=0,
                    flash_off=0
                )
            res.success = True
        except OSError:
            res.success = False
            res.message = "Error setting led flash"

        return res

    def vibrate_controller_cb(self,
                              req: VibratePs4Controller.Request, res):
        if self.state == Ps4ControllerState.DISCONNECTED:
            res.success = False
            res.message = "PS4 controller not connected"
            return res

        if self.state == Ps4ControllerState.LOW_BATTERY:
            res.message = "Low battery"
        
        try:
            if req.type == VibratePs4Controller.Request.RUMBLE_SMALL:
                self.controller.control(
                    rumble_small=req.intensity,
                    rumble_big=None
                )
            else:
                self.controller.control(
                    rumble_small=None,
                    rumble_big=req.intensity
                )
            if self.rumble_timer is not None:
                self.rumble_timer.cancel()
                self.rumble_timer.destroy()
                self.rumble_timer = None

            self.rumble_timer = self.create_timer(
                req.duration_in_ms / 1e3,
                callback=self._on_rumble_stop
            )

            res.success = True
        except OSError:
            res.success = False
            res.message = "Error in setting controller rumble"
        return res

    def _on_rumble_stop(self):
        self.controller.control(rumble_small=0, rumble_big=0)
        if self.rumble_timer is not None:
            self.rumble_timer.cancel()
            self.rumble_timer.destroy()
            self.rumble_timer = None

    def start(self):
        self.get_clock().sleep_for(Duration(seconds=2))

        self.controller = ControllerThread()
        self.backend = HidrawBackend(Logger("backend"))
            
        try:
            device = self.backend.get_hid_device()
            self.controller.setup_device(device)
            self.controller.start()
            self.controller.loop.register_event(
                "device-report", self.cb_report
            )
        except DeviceError as e:
            self.get_logger().error(f"{e}")
            return
        
        # set default color when setup
        self.controller.control(
            led_red=0,
            led_green=0,
            led_blue=255
        )

        self.state = Ps4ControllerState.CONNECTED
        self.get_logger().info("PS4 controller ready to use")

    def stop(self):
        if self.controller is None:
            return
        self.controller.exit()
        self.controller = None

    @staticmethod
    def _normalize_axis_(val, deadzone=0.0):
        """
        Convert a value of [0, 255] to [-1.0, 1.0]
        :param val:
        :param deadzone:
        :return:
        """
        norm_val = 2 * (val - 127.5) / 255
        if abs(norm_val) < deadzone:
            return 0.0
        else:
            return norm_val


def main(args=None):
    rclpy.init(args=args)
    ps4_controller = Ps4Controller()
    multi_thread_executor = MultiThreadedExecutor()
    multi_thread_executor.add_node(ps4_controller)

    try:
        spin(ps4_controller, executor=multi_thread_executor, hz=125)
    except KeyboardInterrupt:
        print("Shutting down...")
        ps4_controller.stop()
        ps4_controller.destroy_node()


if __name__ == "__main__":
    main()
