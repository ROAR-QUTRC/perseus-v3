# v4l_monitor.py
import os
import threading
import pyudev

from message_types import Device

DEV_DIR = "/dev"
V4L_DIR = "/sys/class/video4linux"
VIRTUAL_DEVICE_DIR = "/sys/devices/virtual/video4linux"

_devices = []
_lock = threading.Lock()
_context = None
_observer = None


def _list_devices(hostname, context) -> list[Device]:
    devices: list[Device] = []

    if os.path.isdir(V4L_DIR):
        for entry in os.listdir(V4L_DIR):
            if not entry.startswith("video"):
                continue
            if not _is_video_capture_device(entry, context):
                continue
            with open(os.path.join(V4L_DIR, entry, "name"), "r") as name:
                is_virtual = _is_virtual_video_capture_device(entry)
                if not is_virtual and not _is_usb_video_capture_device(entry, context):
                    continue
                device: Device = Device(
                    name=name.read().strip(),
                    hostname=hostname,
                    dev=os.path.join(DEV_DIR, entry),
                    isVirtual=is_virtual,
                )
                devices.append(device)

    return devices


def _get_udev_device(dev, context):
    if not os.path.exists(os.path.join(DEV_DIR, dev)):
        return None
    device = pyudev.Devices.from_name(context, "video4linux", dev)
    return device


# check if a videoXX string points to a video capture device
def _is_video_capture_device(dev, context):
    device = _get_udev_device(dev, context)
    if device is None:
        return False
    caps = device.properties.get("ID_V4L_CAPABILITIES", "")
    if "capture" not in caps:
        return False
    return True


# need a filter for usb cameras since raspberry pi encoder hardware gets /dev/videoXX entries
def _is_usb_video_capture_device(dev, context):
    device = _get_udev_device(dev, context)
    if device is None:
        return False
    if not _is_video_capture_device(dev, context):
        return False
    return device.find_parent("usb") is not None


def _is_virtual_video_capture_device(dev):
    # Check if the device is a virtual video capture device by looking for its sysfs path
    sysfs_path = os.path.join(VIRTUAL_DEVICE_DIR, dev)
    return os.path.exists(sysfs_path)


# check for changes and call main callback
def _handle_udev_event(hostname, on_change, _):
    with _lock:
        global _devices
        new_devices = _list_devices(hostname, _context)
        changed = new_devices != _devices
        _devices = new_devices

    if changed and on_change:
        on_change(list(_devices))


# start a udev observer which executes a callback on 'video4linux' device changes
# returns the initial list of devices
def start_v4l_monitor(hostname, on_change=None) -> list[Device]:
    global _context, _observer

    if _observer is None:
        _context = pyudev.Context()
        monitor = pyudev.Monitor.from_netlink(_context)
        monitor.filter_by(subsystem="video4linux")

        _observer = pyudev.MonitorObserver(
            monitor,
            callback=lambda device: _handle_udev_event(hostname, on_change, device),
        )
        _observer.start()

    with _lock:
        _devices: list[Device] = list(_list_devices(hostname, _context))

    return list(_devices)


def stop_v4l_monitor():
    global _observer
    if _observer is not None:
        _observer.stop()
        _observer = None
