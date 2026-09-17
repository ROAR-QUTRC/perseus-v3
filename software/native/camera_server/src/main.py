import sys
from typing import Literal
from signal import signal, SIGINT
import socketio
import gi
from time import sleep

from logger import log, enable_debug
from v4l_monitor import start_v4l_monitor, stop_v4l_monitor

gi.require_version("GLib", "2.0")
gi.require_version("GObject", "2.0")
gi.require_version("Gst", "1.0")

# Formatter error E402 is silenced as the gi design pattern requires imports to be after the gi.require_version calls.
# gi.require_version("GLib", "2.0") generates typelib data at runtime.
# pylance throws an error (which we can ignore) on the following line due to this.
# from gi.repository import Gst # noqa: E402 # type: ignore[reportMissingImports]

videoTransformType = Literal[
    "none",
    "clockwise",
    "counterclockwise",
    "rotate-180",
    "horizontal-flip",
    "vertical-flip",
    "upper-left-diagonal",
    "upper-right-diagonal",
    "automatic",
]


def cleanup(sig, frame):
    log("Shutting down camera server...")

    # cleanup here
    stop_v4l_monitor()

    log("Bye!")
    sys.exit(0)


signal(SIGINT, cleanup)


def main():
    # -----------< Parse command line arguments >-----------

    args = sys.argv[1:]
    if "--debug" in args:
        enable_debug()
        args.remove("--debug")
    log("Debug logging enabled", "DEBUG")

    hostname = "localhost"
    port = 3000

    if len(args) > 0:
        # hostname:port passed as command line argument
        if ":" in args[0]:
            hostname, port_str = args[0].split(":", 1)
            try:
                port = int(port_str)
            except ValueError:
                log(f"Invalid port number: {port_str}", "ERROR")
                sys.exit(1)
            if hostname == "":
                hostname = "localhost"
        else:
            hostname = args[0]
    log(f"Connecting to webserver at {hostname}:{port}")

    # ----------< Get hardware information >-----------

    def handle_device_change(devices):
        log(f"Detected camera device change: {devices}", "DEBUG")

    devices = start_v4l_monitor(handle_device_change)
    log(f"Detected camera devices: {devices}")

    # -----------< Connect to web server socket >-----------

    sio = socketio.Client()

    @sio.event
    def camera_event(data):
        handle_camera_event(data)

    while sio.connected is False:
        try:
            sio.connect(f"http://{hostname}:{port}", {})
            log("Connected to web server socket")
        except Exception as e:
            log("Connection error, please check the config. Retrying...", "ERROR")
            log(f"Error: {e}", "DEBUG")
            sleep(1)

    # -----------< Send initial message to web server >-----------

    sio.send({"type": "camera", "action": "group-description"})

    while True:
        sleep(0.0001)

    cleanup(None, None)


def handle_camera_event(data):
    log(f"Received camera event: {data}", "DEBUG")


if __name__ == "__main__":
    main()
