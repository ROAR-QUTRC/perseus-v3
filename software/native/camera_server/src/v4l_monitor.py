# v4l_monitor.py
import os
import threading
import pyudev
from logger import log
from pathlib import Path
import subprocess

DEV_DIR = "/dev/"
BY_ID_DIR = "/dev/v4l/by-id"
V4L_DIR = "/sys/class/video4linux"

_devices = []
_lock = threading.Lock()
_context = None
_observer = None


# TODO:
# This does not clean the symlinks when the camera disconnects.
# Maybe rely on calling a sudo command on a timer to maintain the sudo grace period
# Since we will need to sudo to remove the symlinks but dont want to halt the server
def _fix_duplicate_device_names(server_name: str):

    # Find all video capture devices
    devices = []
    if os.path.isdir(V4L_DIR):
        for entry in os.listdir(V4L_DIR):
            if not entry.startswith("video"):
                continue
            if not _is_video_capture_device(entry, _context):
                continue
            devices.append(os.path.join(DEV_DIR, entry))

    # Remove the ones which are correctly symlinked in /dev/v4l/by-id
    devices_by_id = os.listdir(BY_ID_DIR) if os.path.isdir(BY_ID_DIR) else []
    for dev in devices_by_id:
        by_id = Path(os.path.join(BY_ID_DIR, dev))
        symlink = by_id.resolve()

        if str(symlink) in devices:
            devices.remove(str(symlink))

    # Create symlinks for the remaining devices that don't have by-id symlinks
    if len(devices) > 0:
        log(f"Found {len(devices)} devices without by-id symlinks: {devices}", "DEBUG")
        log("Duplicates found. Attempting to escalate privileges to create symlinks...")
        command_base = ["sudo", "ln", "-s"]
        index = 0
        for dev in devices:
            command = command_base.copy()
            command.append(dev)
            command.append(os.path.join(BY_ID_DIR, f"{server_name}_cam_{index}"))
            try:
                subprocess.run(command, check=True)
                log(f"Created symlink for {dev} as {server_name}_cam_{index}", "DEBUG")
            except subprocess.CalledProcessError as e:
                log(f"Failed to create symlink for {dev}: {e}", "ERROR")
            index += 1


# read the device name from the file /sys/class/video4linux/videoXX/name
def _read_device_name(dev):
    try:
        with open(os.path.join(V4L_DIR, dev, "name"), "r") as f:
            return f.read().strip()
    except (FileNotFoundError, OSError):
        return None


# check if a videoXX string points to a video capture device
def _is_video_capture_device(dev, context):
    device = pyudev.Devices.from_name(context, "video4linux", dev)
    caps = device.properties.get("ID_V4L_CAPABILITIES", "")
    return "capture" in caps


# gets all the video capture devices in /dev/v4l/by-id
def _scan_devices(context):
    devices = []
    if os.path.isdir(BY_ID_DIR):
        for entry in os.listdir(BY_ID_DIR):
            symlink = str(Path(os.path.join(BY_ID_DIR, entry)).resolve()).replace(
                DEV_DIR, ""
            )
            if not symlink.startswith("video"):
                continue
            if not _is_video_capture_device(symlink, context):
                continue

            devices.append(entry)
    return devices


# check for changes and call main callback
def _handle_udev_event(on_change, _):
    with _lock:
        global _devices
        new_devices = _scan_devices(_context)
        changed = new_devices != _devices
        _devices = new_devices

    if changed and on_change:
        on_change(list(_devices))


# start a udev observer which executes a callback on 'video4linux' device changes
# returns the initial list of devices
def start_v4l_monitor(server_name: str, on_change=None) -> list[str]:
    global _context, _observer

    if _observer is None:
        _context = pyudev.Context()
        monitor = pyudev.Monitor.from_netlink(_context)
        monitor.filter_by(subsystem="video4linux")

        _fix_duplicate_device_names(server_name)

        _observer = pyudev.MonitorObserver(
            monitor, callback=lambda device: _handle_udev_event(on_change, device)
        )
        _observer.start()

    with _lock:
        _devices = list(_scan_devices(_context))

    return list(_devices)


def stop_v4l_monitor():
    global _observer
    if _observer is not None:
        _observer.stop()
        _observer = None
