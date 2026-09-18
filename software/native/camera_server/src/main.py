import json
import sys
from typing import Literal
from signal import signal, SIGINT
import socketio
import gi
from time import sleep
from pydantic import ValidationError, BaseModel

from logger import log, enable_debug
from v4l_monitor import start_v4l_monitor, stop_v4l_monitor
from message_types import CameraEventData, CameraEventType, GstInstance, Resolution

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

group_description = CameraEventType(
    type="camera", action="group-description", data=CameraEventData()
)


def main():
    # -----------< Handle Graceful Shutdown >-----------

    # none of the arguments are needed
    def cleanup(*_):
        log("Shutting down camera server...")

        # cleanup here
        stop_v4l_monitor()

        log("Bye!")
        sys.exit(0)

    signal(SIGINT, cleanup)

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

    devices: dict[str, str] = {}  # {videoXX: name}

    def handle_device_change(new_devices: dict[str, str]):
        # Update the devices dictionary with the new devices and remove old ones
        for dev in list(devices.keys()):
            if dev not in new_devices:
                log(f"Camera device removed: {dev} -> {devices[dev]}")
                sio.send(
                    {
                        "type": "camera",
                        "action": "device-disconnect",
                        "data": {"devices": [dev]},
                    }
                )
                del devices[dev]
        for dev, name in new_devices.items():
            if dev not in devices:
                log(f"Camera device added: {dev} -> {name}")
                devices[dev] = name
                group_description.data.deviceNames = devices
                sio.send(group_description.model_dump(exclude_none=True))

    # start monitor and get initial device list
    devices = start_v4l_monitor(handle_device_change)
    for dev, name in devices.items():
        log(f"Detected camera device: {dev} -> {name}")

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

    # -----------< Handle Socket Events >-----------

    group_description.data.deviceNames = devices
    sio.send(group_description.model_dump(exclude_none=True))

    gst_instances: dict[str, GstInstance] = {}  # {videoXX: GstInstance}

    def start_stream_for_device(
        dev: str, resolution: dict[str, int] | Resolution, transform: str
    ):
        log(
            f"Starting stream for device: {dev} with resolution: {resolution} and transform: {transform}",
            "DEBUG",
        )

    def handle_camera_event(event_payload: CameraEventType):
        try:
            event = CameraEventType.model_validate(event_payload)
        except ValidationError as e:
            log(f"Invalid camera event payload: {e}", "ERROR")
            return
        
        # ignore if device not owned by this server
        if event.data:
            if event.data.devices and event.data.devices[0] not in devices:
                log(f"Ignoring camera event: \n{json.dumps(event.model_dump(), indent=4)}", "DEBUG")
                return
        log(f"Handling camera event: \n{json.dumps(event.model_dump(), indent=4)}", "DEBUG")

        match event.action:
            case "request-groups":
                group_description.data.deviceNames = devices
                sio.send(group_description.model_dump(exclude_none=True))
            case "request-stream":
                log(f"Request stream for device: {event.data.devices}", "DEBUG")
                dev = event.data.devices[0] if event.data.devices else None
                if dev is None:
                    return
                instance = gst_instances.get(dev, None)

                if (
                    event.data.forceRestart and instance is not None
                ) or instance is None:
                    log(f"Force restarting stream for device: {dev}", "DEBUG")
                    # TODO: Gracefully stop the existing stream
                    del gst_instances[dev]

                start_stream = True

                if instance is not None:
                    # Check if any preferences have changed
                    if (
                        event.data.resolution != instance.resolution
                        or event.data.transform != instance.transform
                    ):
                        log(
                            f"Preferences changed for device: {dev}, restarting stream",
                            "DEBUG",
                        )
                        # TODO: Gracefully stop the existing stream
                        del gst_instances[dev]
                    else:
                        log(
                            f"Stream already running for device: {dev}, ignoring request",
                            "DEBUG",
                        )
                        start_stream = False

                if start_stream:
                    # Validate arguments
                    resolution = (
                        event.data.resolution
                        if event.data.resolution
                        else {"width": 640, "height": 480}
                    )
                    if (
                        not isinstance(resolution, dict)
                        or "width" not in resolution
                        or "height" not in resolution
                    ):
                        log(
                            f"Invalid resolution: {resolution}, defaulting to 640x480",
                            "ERROR",
                        )
                        resolution = {"width": 640, "height": 480}
                    else:
                        if not isinstance(resolution["width"], int) or not isinstance(
                            resolution["height"], int
                        ):
                            log(
                                f"Invalid resolution values: {resolution}, defaulting to 640x480",
                                "ERROR",
                            )
                            resolution = {"width": 640, "height": 480}
                    transform = event.data.transform if event.data.transform else "none"
                    if transform not in videoTransformType.__args__:
                        log(
                            f"Invalid transform type: {transform}, defaulting to 'none'",
                            "ERROR",
                        )
                        transform = "none"
                    start_stream_for_device(dev, resolution, transform)

            case "group-description":
                # ignore self sent messages
                pass
            case _:
                log(f"Unhandled camera event: \n{json.dumps(event.model_dump(), indent=4)}", "DEBUG")

    sio.wait()

    cleanup(None, None)


if __name__ == "__main__":
    main()
