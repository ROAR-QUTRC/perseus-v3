# v4l_monitor.py
import os
import threading
import pyudev
import subprocess

from server.logger import log
from message_types import DeviceInfo

DEV_DIR = "/dev"
BY_ID_DIR = "/dev/v4l/by-id"
V4L_DIR = "/sys/class/video4linux"
VIRTUAL_DEVICE_DIR = "/sys/devices/virtual/video4linux"
VIRTUAL_DEVICE_COUNT = 4  # Number of virtual devices to create
# Offset virtual devices numbers to avoid collisions when replugging cameras
VIRTUAL_DEVICE_OFFSET = 64

_devices = []
_lock = threading.Lock()
_context = None
_observer = None


def _create_virtual_devices(device_count: int):
    log(f"Creating {device_count} virtual video devices...", "DEBUG")
    existing_virtual_devices = _list_virtual_devices()
    device_numbers = []
    for i in range(device_count):
        video_id = i + VIRTUAL_DEVICE_OFFSET
        if f"video{video_id}" in existing_virtual_devices:
            continue
        device_numbers.append(str(video_id))

    if len(device_numbers) == 0:
        return

    command = [
        "sudo",
        "modprobe",
        "v4l2loopback",
        f"video_nr={','.join(device_numbers)}",
    ]

    try:
        subprocess.run(command, check=True)
        # check video devices were created since we get no command output
        existing_virtual_devices = _list_virtual_devices()
        missing_devices = [
            f"video{num}"
            for num in device_numbers
            if f"video{num}" not in existing_virtual_devices
        ]
        if missing_devices:
            log(
                f"Failed to create virtual video devices: {', '.join(missing_devices)}",
                "ERROR",
            )
    except subprocess.CalledProcessError as e:
        log(f"Failed to create virtual video devices: {e}", "ERROR")


def _list_virtual_devices():
    # Dont need to check if devices are video capture
    # since all virtual devices are video capture
    virtual_devices = []
    if os.path.isdir(VIRTUAL_DEVICE_DIR):
        for entry in os.listdir(VIRTUAL_DEVICE_DIR):
            if entry.startswith("video"):
                virtual_devices.append(entry)
    return virtual_devices


# check if a videoXX string points to a video capture device
def _is_video_capture_device(dev, context):
    # check if file exists in /dev/videoXX
    if not os.path.exists(os.path.join(DEV_DIR, dev)):
        return False
    device = pyudev.Devices.from_name(context, "video4linux", dev)
    caps = device.properties.get("ID_V4L_CAPABILITIES", "")
    return "capture" in caps


# gets all the video capture devices in /sys/class/video4linux
# returns a list of DeviceInfo
def _list_devices(context) -> list[DeviceInfo]:
    devices = []
    if os.path.isdir(V4L_DIR):
        for entry in os.listdir(V4L_DIR):
            if not entry.startswith("video"):
                continue
            if not _is_video_capture_device(entry, context):
                continue
            # device name is stored in V4L_DIR/videoXX/name
            name_file = os.path.join(V4L_DIR, entry, "name")
            if os.path.isfile(name_file):
                with open(name_file, "r") as f:
                    # only save devices with names (should be all of them)
                    devices.append(DeviceInfo(name=f.read().strip(), device=entry))
    return devices


def _list_physical_devices(context) -> list[DeviceInfo]:
    devices = _list_devices(context)
    virtual_devices = _list_virtual_devices()
    return [dev for dev in devices if dev.device not in virtual_devices]


# check for changes and call main callback
def _handle_udev_event(on_change, _):
    with _lock:
        global _devices
        new_devices = _list_physical_devices(_context)
        changed = new_devices != _devices
        _devices = new_devices

    if changed and on_change:
        on_change(list(_devices))


# start a udev observer which executes a callback on 'video4linux' device changes
# returns the initial list of devices
def start_v4l_monitor(server_name: str, on_change=None) -> list[DeviceInfo]:
    global _context, _observer

    if _observer is None:
        _context = pyudev.Context()
        monitor = pyudev.Monitor.from_netlink(_context)
        monitor.filter_by(subsystem="video4linux")

        _create_virtual_devices(VIRTUAL_DEVICE_COUNT)
        # _fix_duplicate_device_names(server_name)

        _observer = pyudev.MonitorObserver(
            monitor, callback=lambda device: _handle_udev_event(on_change, device)
        )
        _observer.start()

    with _lock:
        _devices = list(_list_physical_devices(_context))

    return list(_devices)


def stop_v4l_monitor():
    global _observer
    if _observer is not None:
        _observer.stop()
        _observer = None
