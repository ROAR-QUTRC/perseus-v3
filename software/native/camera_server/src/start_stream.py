from dataclasses import dataclass, field
from typing import Optional

from message_types import CameraEventType, VideoTransformType
from logger import log
import gi
import threading

gi.require_version("GLib", "2.0")
gi.require_version("GObject", "2.0")
gi.require_version("Gst", "1.0")

# Formatter error E402 is silenced as the gi design pattern requires imports to be after the gi.require_version calls.
# gi.require_version("GLib", "2.0") generates typelib data at runtime.
# pylance throws an error (which we can ignore) on the following line due to this.
from gi.repository import Gst, GLib # noqa: E402 # type: ignore[reportMissingImports]

@dataclass
class GstInstance:
    device: str
    # Unique name used to distinguish videoXX on multiple devices
    device_meta_name: str
    width: int
    height: int
    transform: VideoTransformType
    pipeline: Optional[Gst.Pipeline] = field(repr=False)
    file: Optional[str] = None

Gst.init()

gst_lock = threading.Lock()
gst_instances: dict[str, GstInstance] = {}  # {videoXX: GstInstance}

glib_loop: GLib.MainLoop | None = None
glib_thread: threading.Thread | None = None

def start_gst_thread():
    global glib_loop, glib_thread
    if glib_loop is not None and glib_thread is not None and glib_thread.is_alive():
        return # Already running, shouldnt get here
    
    glib_loop = GLib.MainLoop()
    glib_thread = threading.Thread(target=glib_loop.run, daemon=True)
    glib_thread.start()

def stop_gst_main_loop():
    if glib_loop is not None:
        glib_loop.quit()
    if glib_thread is not None:
        glib_thread.join(timeout=5)

def _create_element(element_type: str, name: str) -> Gst.Element | None:
    element = Gst.ElementFactory.make(element_type, name)
    if not element:
        log(f"Failed to create GStreamer element: {element_type} ({name})", "ERROR")
    return element

def _instance_changed(old:GstInstance, new:GstInstance) -> bool:
    return (
        old.width != new.width or
        old.height != new.height or
        old.transform != new.transform or
        old.file != new.file
    )

def start_stream(event: CameraEventType, web_server_ip: str):
    log(f"Request stream for device: {event.target}", "DEBUG")

    # validate event
    device = event.target.dev if event.target else None
    if device is None:
        log("No device specified in event", "ERROR")
        return
    device_meta_name = f"{device}-{event.target.serverName}" if event.target else None
    if device_meta_name is None:
        log("No device meta name could be constructed", "ERROR")
        return
    new_instance = GstInstance(
        device=device,
        device_meta_name=device_meta_name,
        width=event.data.resolution["width"] if event.data and event.data.resolution else 640,
        height=event.data.resolution["height"] if event.data and event.data.resolution else 480,
        transform=event.data.transform if event.data and event.data.transform else "none",
        file=event.data.file if event.data and event.data.file else None,
        pipeline=None
    )
    force_restart = event.data.forceRestart if event.data and event.data.forceRestart else False

    with gst_lock:
        # skip is stream is already running
        old_instance = gst_instances.get(device, None)
        if old_instance is not None and not force_restart and not _instance_changed(old_instance, new_instance):
            log(f"Stream is already running for device: {device_meta_name} with correct settings, ignoring request", "DEBUG")
            return

        # Remove the old instance if it exists
        if old_instance is not None:
            log(f"Stopping existing stream for device: {device_meta_name}", "DEBUG")
            if old_instance.pipeline is not None:
                old_instance.pipeline.set_state(Gst.State.NULL)
            del gst_instances[device]

    # Create the elements
    source = _create_element("v4l2src", "source")
    convert = _create_element("videoconvert", "convert")
    sink = _create_element("autovideosink", "sink")

    # Create the empty pipeline
    new_instance.pipeline = Gst.Pipeline.new(device_meta_name)

    if not new_instance.pipeline or not source or not convert or not sink:
        log("Not all elements could be created.", "ERROR")
        return

    # Build the pipeline. Note that we are NOT linking the source at this
    # point. We will do it later.
    new_instance.pipeline.add(source, convert, sink)
    if not source.link(convert) or not convert.link(sink):
        log("Elements could not be linked.", "ERROR")
        return

    # Set the device to use
    source.set_property("device", f"/dev/{device}")

    # Start playing
    ret = new_instance.pipeline.set_state(Gst.State.PLAYING)
    if ret == Gst.StateChangeReturn.FAILURE:
        log("Unable to set the pipeline to the playing state.", "ERROR")
        return

    # Listen to the bus
    bus = new_instance.pipeline.get_bus()
    bus.add_signal_watch()
    bus.connect("message", lambda bus, msg: log(f"Message from bus ({device}): {msg}", "DEBUG"), None)

    with gst_lock:
        gst_instances[device] = new_instance
    

































    
    # log(f"Request stream for device: {event.data.devices}", "DEBUG")
    # dev = event.data.devices[0] if event.data.devices else None
    # if dev is None:
    #     return
    # instance = gst_instances.get(dev, None)

    # if (
    #     event.data.forceRestart and instance is not None
    # ) or instance is None:
    #     log(f"Force restarting stream for device: {dev}", "DEBUG")
    #     # TODO: Gracefully stop the existing stream
    #     del gst_instances[dev]

    # start_stream = True

    # if instance is not None:
    #     # Check if any preferences have changed
    #     if (
    #         event.data.resolution != instance.resolution
    #         or event.data.transform != instance.transform
    #     ):
    #         log(
    #             f"Preferences changed for device: {dev}, restarting stream",
    #             "DEBUG",
    #         )
    #         # TODO: Gracefully stop the existing stream
    #         del gst_instances[dev]
    #     else:
    #         log(
    #             f"Stream already running for device: {dev}, ignoring request",
    #             "DEBUG",
    #         )
    #         start_stream = False

    # if start_stream:
    #     # Validate arguments
    #     resolution = (
    #         event.data.resolution
    #         if event.data.resolution
    #         else {"width": 640, "height": 480}
    #     )
    #     if (
    #         not isinstance(resolution, dict)
    #         or "width" not in resolution
    #         or "height" not in resolution
    #     ):
    #         log(
    #             f"Invalid resolution: {resolution}, defaulting to 640x480",
    #             "ERROR",
    #         )
    #         resolution = {"width": 640, "height": 480}
    #     else:
    #         if not isinstance(resolution["width"], int) or not isinstance(
    #             resolution["height"], int
    #         ):
    #             log(
    #                 f"Invalid resolution values: {resolution}, defaulting to 640x480",
    #                 "ERROR",
    #             )
    #             resolution = {"width": 640, "height": 480}
    #     transform = event.data.transform if event.data.transform else "none"
    #     if transform not in VideoTransformType.__args__:
    #         log(
    #             f"Invalid transform type: {transform}, defaulting to 'none'",
    #             "ERROR",
    #         )
    #         transform = "none"
    #     start_stream_for_device(dev, resolution, transform)
