# v4l_monitor.py
import os
import threading
import pyudev

V4L_DIR = "/sys/class/video4linux"

_devices = {}
_lock = threading.Lock()
_context = None
_observer = None


def _read_device_name(dev):
    try:
        with open(os.path.join(V4L_DIR, dev, "name"), "r") as f:
            return f.read().strip()
    except (FileNotFoundError, OSError):
        return None


def _is_video_capture_device(dev, context):
    device = pyudev.Devices.from_name(context, "video4linux", dev)
    caps = device.properties.get("ID_V4L_CAPABILITIES", "")
    return "capture" in caps


def _scan_devices(context):
    devices = {}
    if os.path.isdir(V4L_DIR):
        for entry in os.listdir(V4L_DIR):
            if not entry.startswith("video"):
                continue
            if not _is_video_capture_device(entry, context):
                continue
            name = _read_device_name(entry)
            if name is not None:
                devices[entry] = name
    return devices


def _handle_udev_event(on_change, _):
    with _lock:
        new_devices = _scan_devices(_context)
        changed = new_devices != _devices
        _devices.clear()
        _devices.update(new_devices)
        snapshot = dict(_devices)

    if changed and on_change:
        on_change(snapshot)


def start_v4l_monitor(on_change=None) -> dict[str, str]:
    """
    Scans /sys/class/video4linux, starts a background udev event listener,
    and returns the current {videoXX: name} map immediately.

    on_change(devices_dict) is called from pyudev's observer thread every
    time the device set changes. If you need to touch anything not
    thread-safe (a GLib main loop, an asyncio loop, a GStreamer pipeline),
    marshal inside on_change via GLib.idle_add / loop.call_soon_threadsafe
    rather than acting on it directly.
    """
    global _context, _observer

    if _observer is None:
        _context = pyudev.Context()
        monitor = pyudev.Monitor.from_netlink(_context)
        monitor.filter_by(subsystem="video4linux")

        _observer = pyudev.MonitorObserver(
            monitor, callback=lambda device: _handle_udev_event(on_change, device)
        )
        _observer.start()

    with _lock:
        _devices.clear()
        _devices.update(_scan_devices(_context))
        current = dict(_devices)

    return current


def get_v4l_devices():
    with _lock:
        return dict(_devices)


def stop_v4l_monitor():
    global _observer
    if _observer is not None:
        _observer.stop()
        _observer = None
