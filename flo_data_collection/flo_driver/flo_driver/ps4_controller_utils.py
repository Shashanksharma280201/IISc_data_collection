import fcntl
import os

from io import FileIO

from evdev import InputDevice
from pyudev import Context

from ds4drv.backend import Backend
from ds4drv.exceptions import DeviceError
from ds4drv.device import DS4Device
from ds4drv.utils import zero_copy_slice


IOC_RW = 3221243904
HIDIOCSFEATURE = lambda size: IOC_RW | (0x06 << 0) | (size << 16)
HIDIOCGFEATURE = lambda size: IOC_RW | (0x07 << 0) | (size << 16)


class HidrawDS4Device(DS4Device):
    def __init__(self, name, addr, type, hidraw_device, event_device):
        try:
            self.report_fd = os.open(hidraw_device, os.O_RDWR | os.O_NONBLOCK)
            self.fd = FileIO(self.report_fd, "rb+", closefd=False)
            self.input_device = InputDevice(event_device)
            self.input_device.grab()
        except (OSError, IOError) as err:
            raise DeviceError(err)

        self.buf = bytearray(self.report_size)

        super(HidrawDS4Device, self).__init__(name, addr, type)

    def read_report(self):
        try:
            ret = self.fd.readinto(self.buf)
        except IOError:
            return

        # Disconnection
        if ret == 0:
            return

        # Invalid report size or id, just ignore it
        if ret < self.report_size or self.buf[0] != self.valid_report_id:
            return False

        if self.type == "bluetooth":
            # Cut off bluetooth data
            buf = zero_copy_slice(self.buf, 2)
        else:
            buf = self.buf

        return self.parse_report(buf)

    def read_feature_report(self, report_id, size):
        op = HIDIOCGFEATURE(size + 1)
        buf = bytearray(size + 1)
        buf[0] = report_id

        return fcntl.ioctl(self.fd, op, bytes(buf))

    def write_report(self, report_id, data):
        hid = bytearray((report_id,))
        self.fd.write(hid + data)

    def close(self):
        try:
            # Reset LED to original hidraw pairing colour.
            self.set_led(0, 0, 1)

            self.fd.close()
            self.input_device.ungrab()
        except IOError:
            pass


class HidrawBluetoothDS4Device(HidrawDS4Device):
    __type__ = "bluetooth"

    report_size = 78
    valid_report_id = 0x11

    def set_operational(self):
        self.read_feature_report(0x02, 37)


class HidrawUSBDS4Device(HidrawDS4Device):
    __type__ = "usb"

    report_size = 64
    valid_report_id = 0x01

    def set_operational(self):
        # Get the bluetooth MAC
        addr = self.read_feature_report(0x81, 6)[1:]
        addr = ["{0:02x}".format(c) for c in bytearray(addr)]
        addr = ":".join(reversed(addr)).upper()

        self.device_name = "{0} {1}".format(addr, self.device_name)
        self.device_addr = addr


HID_DEVICES = {
    "Sony Interactive Entertainment Wireless Controller": HidrawUSBDS4Device,
    "Sony Computer Entertainment Wireless Controller": HidrawUSBDS4Device,
    "Wireless Controller": HidrawBluetoothDS4Device,
}


class HidrawBackend(Backend):
    __name__ = "hidraw"

    def get_hid_device(self):
        context = Context()
        for hidraw_device in context.list_devices(subsystem="hidraw"):
            hid_device = hidraw_device.parent
            if hid_device.subsystem != "hid":
                continue

            cls = HID_DEVICES.get(hid_device.get("HID_NAME"))
            if not cls:
                continue
            
            event_device_path = ""
            for child in hid_device.parent.children:
                event_device = child.get("DEVNAME", "")
                if event_device.startswith("/dev/input/event"):
                    for key in child.keys():
                        if "JOYSTICK" in key:
                            event_device_path = event_device
                            break 
            try:
                device_addr = hid_device.get("HID_UNIQ", "").upper()
                if device_addr:
                    device_name = f"{device_addr} {hidraw_device.sys_name}"
                else:
                    device_name = hidraw_device.sys_name

                return cls(name=device_name,
                          addr=device_addr,
                          type=cls.__type__,
                          hidraw_device=hidraw_device.device_node,
                          event_device=event_device_path)

            except DeviceError as err:
                raise DeviceError(f"Error connecting to device : {err}")