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
VIRTUAL_DEVICE_DIR = "/sys/devices/virtual/video4linux"
VIRTUAL_DEVICE_COUNT = 4  # Number of virtual devices to create
VIRTUAL_DEVICE_OFFSET = (
    32  # Offset virtual devices numbers to avoid collisions when repluggin cameras
)

_devices = []
_lock = threading.Lock()
_context = None
_observer = None


# TODO:
# This does not clean the symlinks when the camera disconnects.
# Maybe rely on calling a sudo command on a timer to maintain the sudo grace period
# Since we will need to sudo to remove the symlinks but dont want to halt the server
def _fix_duplicate_device_names(server_name: str):

    # Find all the real video capture devices
    virtual_devices = list_virtual_devices()
    real_devices = []
    if os.path.isdir(V4L_DIR):
        for entry in os.listdir(V4L_DIR):
            if not entry.startswith("video"):
                continue
            if not _is_video_capture_device(entry, _context):
                continue
            if entry in virtual_devices:
                continue  # skip virtual devices
            real_devices.append(os.path.join(DEV_DIR, entry))

    # remove bad links and record current symlinks to real devices in a map
    symlinks_map = {}
    if os.path.isdir(BY_ID_DIR):
        for entry in os.listdir(BY_ID_DIR):
            by_id = Path(os.path.join(BY_ID_DIR, entry))
            by_id_str = str(by_id)
            real_device_dir = by_id.resolve()
            real_device_path = str(real_device_dir)
            if _is_video_capture_device(
                real_device_path.replace(DEV_DIR, ""), _context
            ):
                remove_symlink = False
                if (
                    not real_device_dir.exists()
                    or real_device_path.replace(DEV_DIR, "") in virtual_devices
                ):
                    log(
                        f"Bad symlink detected: {by_id} -> {real_device_dir}, removing...",
                        "DEBUG",
                    )
                    remove_symlink = True
                print(
                    real_device_path in symlinks_map.values(),
                    real_device_path.replace(DEV_DIR, ""),
                )
                if real_device_path in symlinks_map.values():
                    log(f"Dupe detected {by_id} -> {real_device_dir}", "ERROR")
                    print(
                        by_id_str,
                        by_id_str.replace(BY_ID_DIR, ""),
                        f"{server_name}_cam_",
                    )
                    if by_id_str.replace(BY_ID_DIR + "/", "").startswith(
                        f"{server_name}_cam_"
                    ):
                        remove_symlink = True
                    # TODO: if the the good symlink is found last then it will not be removed
                    #       otherwise find the bad one and remove it instead
                    # else:
                    #     # must find other device and remove it instead
                    #     # print(real_device_path)
                    #     # index_of_duplicate = list(symlinks_map.values()).index(real_device_path)
                    #     # by_id = Path(os.path.join(BY_ID_DIR, list(symlinks_map.keys())[index_of_duplicate]))
                    #     # del symlinks_map[list(symlinks_map.keys())[index_of_duplicate]]
                    #     # remove_symlink = True
                if remove_symlink:
                    try:
                        subprocess.run(["sudo", "rm", by_id_str], check=True)
                        log(f"Removed symlink {by_id}", "DEBUG")
                    except Exception as e:
                        log(f"Failed to remove symlink {by_id}: {e}", "ERROR")
                    continue
                # map human readable id to real device path
                symlinks_map[entry] = real_device_path

    # create a symlink for each real_device that isnt in the symlinks_map
    index = 0
    for real_device in real_devices:
        if real_device not in symlinks_map.values():
            symlink_name = f"{server_name}_cam_{index}"
            symlink_path = os.path.join(BY_ID_DIR, symlink_name)
            log(f"Creating symlink {symlink_path} -> {real_device}", "DEBUG")
            try:
                subprocess.run(
                    ["sudo", "ln", "-s", real_device, symlink_path], check=True
                )
                log(f"Created symlink {symlink_path} -> {real_device}", "DEBUG")
                index += 1
            except subprocess.CalledProcessError as e:
                log(
                    f"Failed to create symlink {symlink_path} -> {real_device}: {e}",
                    "ERROR",
                )


def _create_virtual_devices(device_count: int):
    log(f"Creating {device_count} virtual video devices...", "DEBUG")
    existing_virtual_devices = list_virtual_devices()
    device_numbers = []
    for i in range(device_count):
        video_id = i + VIRTUAL_DEVICE_OFFSET
        if f"video{video_id}" in existing_virtual_devices:
            continue
        device_numbers.append(str(video_id))

    if len(device_numbers) == 0:
        return

    command_base = [
        "sudo",
        "modprobe",
        "v4l2loopback",
        f"video_nr={','.join(device_numbers)}",
    ]
    try:
        subprocess.run(command_base, check=True)
        # check video devices were created since we get no command output
        existing_virtual_devices = list_virtual_devices()
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


def list_virtual_devices():
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

        _create_virtual_devices(4)  # Create 4 virtual devices
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
