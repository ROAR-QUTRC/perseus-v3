import json
import sys
from signal import signal, SIGINT
import socketio
from time import sleep
from pydantic import ValidationError

from logger import log, enable_debug
from v4l_monitor import start_v4l_monitor, stop_v4l_monitor
from message_types import (
    CameraEventType,
    Device,
)
from start_stream import start_stream, start_gst_thread

group_description = CameraEventType(type="camera", action="group-description")


def main():

    # region -----------< Handle Graceful Shutdown >-----------

    # none of the arguments are needed
    def cleanup(*_):
        log("Shutting down camera server...")

        # cleanup here
        stop_v4l_monitor()

        log("Bye!")
        sys.exit(0)

    signal(SIGINT, cleanup)
    # endregion

    # region -----------< Parse command line arguments >-----------

    args = sys.argv[1:]
    if "--debug" in args:
        enable_debug()
        args.remove("--debug")
    log("Debug logging enabled", "DEBUG")

    hostname = "localhost"
    port = 3000
    server_name = "cam_server"

    with open("config.json", "r") as f:
        config = json.load(f)
        server_name = config.get("name", "cam_server")
        if "hostname" in config:
            hostname = config["hostname"]
        if "port" in config:
            port = config["port"]

    if len(args) > 0:
        # hostname:port passed as command line argument
        if ":" in args[0]:
            hostname_str, port_str = args[0].split(":", 1)
            try:
                port = int(port_str)
            except ValueError:
                log(f"Invalid port number: {port_str}", "ERROR")
                sys.exit(1)
            if hostname_str != "":
                hostname = hostname_str
        elif len(args) == 1:
            hostname = args[0]
    log(f"{server_name} connecting to webserver at {hostname}:{port}")
    # endregion

    # region -----------< Get hardware information >-----------

    devices: list[str] = []  # [videoXX]
    device_names: dict[str, str] = {}  # {videoXX: name}

    def handle_device_change(new_devices: dict[str, str]):
        # Update the devices dictionary with the new devices and remove old ones
        for dev in devices:
            if dev not in new_devices:
                log(f"Camera device removed: {dev} -> {device_names[dev]}")
                sio.send(
                    {
                        "type": "camera",
                        "action": "device-disconnect",
                        "target": {"dev": dev, "serverName": server_name},
                    }
                )
                devices.remove(dev)
                del device_names[dev]
        additions: list[Device] = []
        for dev, name in new_devices.items():
            if dev not in devices:
                log(f"Camera device added: {dev} -> {name}")
                devices.append(dev)
                device_names[dev] = name
                additions.append(Device(dev=dev, name=name, serverName=server_name))
        if len(additions) > 0:
            group_description.devices = additions
            sio.send(group_description.model_dump(exclude_none=True))

    # start monitor and get initial device list
    initial_devices = start_v4l_monitor(handle_device_change)
    for dev, name in initial_devices.items():
        devices.append(dev)
        device_names[dev] = name
        log(f"Detected camera device: {dev} -> {name}")
    # endregion

    # region -----------< Start GStreamer Thread >-----------
    start_gst_thread()
    # endregion

    # region -----------< Connect to web server socket >-----------

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
    # endregion

    # region -----------< Handle Socket Events >-----------

    # Send initial group description to the server
    group_description.devices = [
        Device(dev=dev, name=name, serverName=server_name)
        for dev, name in device_names.items()
    ]
    sio.send(group_description.model_dump(exclude_none=True))

    def handle_camera_event(event_payload: CameraEventType):
        try:
            event = CameraEventType.model_validate(event_payload)
        except ValidationError as e:
            log(f"Invalid camera event payload: {e}", "ERROR")
            return

        log(
            f"Handling camera event: \n{json.dumps(event.model_dump(), indent=4)}",
            "DEBUG",
        )

        match event.action:
            case "request-groups":
                # Broadcast handled by all camera servers
                group_description.devices = [
                    Device(dev=dev, name=name, serverName=server_name)
                    for dev, name in device_names.items()
                ]
                sio.send(group_description.model_dump(exclude_none=True))
            case "request-stream":
                if event.target is None:
                    log("No target device specified for request-stream action", "ERROR")
                    return
                if event.target.serverName != server_name:
                    log(
                        f"Request stream for device: {event.target.dev} ignored, not for this server ({server_name})",
                        "DEBUG",
                    )
                    return
                if event.target.dev not in devices:
                    log(
                        f"Request stream for device: {event.target.dev} ignored, not a valid device",
                        "DEBUG",
                    )
                    return
                start_stream(event, hostname)

            case "group-description":
                # ignore self sent messages
                pass
            case _:
                log(
                    f"Unhandled camera event: \n{json.dumps(event.model_dump(), indent=4)}",
                    "DEBUG",
                )

    sio.wait()

    cleanup(None, None)


if __name__ == "__main__":
    main()
