from dataclasses import dataclass, field
import json
import os
from typing import Optional, cast

from message_types import CameraEventType, VideoTransformType
from logger import log, log_level_type
import gi
import threading

from v4l_monitor import BY_ID_DIR

os.environ["FC_DEBUG"] = "-1"

gi.require_version("GLib", "2.0")
gi.require_version("GObject", "2.0")
gi.require_version("Gst", "1.0")

# Formatter error E402 is silenced as the gi design pattern requires imports to be after the gi.require_version calls.
# gi.require_version("GLib", "2.0") generates typelib data at runtime.
# pylance throws an error (which we can ignore) on the following line due to this.
from gi.repository import Gst, GLib  # noqa: E402 # type: ignore[reportMissingImports]


@dataclass
class GstInstance:
    device: str
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
        return  # Already running, shouldnt get here

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


def _instance_changed(old: GstInstance, new: GstInstance) -> bool:
    return (
        old.width != new.width
        or old.height != new.height
        or old.transform != new.transform
        or old.file != new.file
    )

def _on_gst_message(_, message: Gst.Message, device: str):
    t = message.type
    type_name = Gst.MessageType.get_name(t)

    src = message.src
    src_name = src.get_name() if hasattr(src, "get_name") else None

    detail = ""
    log_level = "INFO"
    try:
        if t == Gst.MessageType.STATE_CHANGED:
            if src_name != device:
                log_level = "DEBUG"
            old, new, pending = message.parse_state_changed()
            old_s = Gst.Element.state_get_name(old)
            new_s = Gst.Element.state_get_name(new)
            detail = f"{old_s} -> {new_s}"
            if pending != Gst.State.VOID_PENDING:
                detail += f" (pending {Gst.Element.state_get_name(pending)})"

        elif t == Gst.MessageType.STREAM_STATUS:
            status_type, owner = message.parse_stream_status()
            owner_name = owner.get_name() if hasattr(owner, "get_name") else str(owner)
            detail = f"{status_type.value_nick} owner={owner_name}"

        elif t == Gst.MessageType.NEW_CLOCK:
            clock = message.parse_new_clock()
            detail = f"clock={clock.get_name() if clock else 'None'}"

        elif t == Gst.MessageType.STREAM_START:
            ok, group_id = message.parse_group_id()
            detail = f"group-id={group_id}" if ok else ""

        elif t == Gst.MessageType.ASYNC_DONE:
            running_time = message.parse_async_done()
            detail = "running-time=none" if running_time == Gst.CLOCK_TIME_NONE else f"running-time={running_time}"

        elif t == Gst.MessageType.LATENCY:
            detail = "recalculate latency"

        elif t == Gst.MessageType.NEED_CONTEXT:
            _, context_type = message.parse_context_type()
            detail = f"context-type={context_type}"

        elif t == Gst.MessageType.HAVE_CONTEXT:
            context = message.parse_have_context()
            detail = f"context-type={context.get_context_type()}"

        elif t == Gst.MessageType.TAG:
            taglist = message.parse_tag()
            detail = taglist.to_string()

        elif t in (Gst.MessageType.ERROR, Gst.MessageType.WARNING, Gst.MessageType.INFO):
            parse_fn = {
                Gst.MessageType.ERROR: message.parse_error,
                Gst.MessageType.WARNING: message.parse_warning,
                Gst.MessageType.INFO: message.parse_info,
            }[t]
            log_level = cast(log_level_type, {
                Gst.MessageType.ERROR: "ERROR",
                Gst.MessageType.WARNING: "WARN",
                Gst.MessageType.INFO: "INFO",
            }[t])
            gerror, debug = parse_fn()
            detail = f"{gerror.message}" + (f" | {debug}" if debug else "")

        elif t == Gst.MessageType.EOS:
            log_level = "WARN"
            detail = ""

        else:
            structure = message.get_structure()
            detail = structure.to_string() if structure is not None else ""

    except Exception:
        # fall back to a raw structure dump if parsing failed for any reason
        structure = message.get_structure()
        detail = structure.to_string() if structure is not None else ""

    line = f"{device} {type_name}"
    if src_name and src_name != device:
        line += f" [{src_name}]"
    if detail:
        line += f": {detail}"
    log(line, log_level)

def start_stream(event: CameraEventType, web_server_ip: str):
    log(f"Request stream for device: {event.target}", "DEBUG")

    # validate event
    device = event.target if event.target else None
    if device is None:
        log("No device specified in event", "ERROR")
        return
    new_instance = GstInstance(
        device=device,
        width=event.data.resolution["width"]
        if event.data and event.data.resolution
        else 640,
        height=event.data.resolution["height"]
        if event.data and event.data.resolution
        else 480,
        transform=event.data.transform
        if event.data and event.data.transform
        else "none",
        file=event.data.file if event.data and event.data.file else None,
        pipeline=None,
    )
    force_restart = (
        event.data.forceRestart if event.data and event.data.forceRestart else False
    )

    with gst_lock:
        # skip is stream is already running
        old_instance = gst_instances.get(device, None)
        if (
            old_instance is not None
            and not force_restart
            and not _instance_changed(old_instance, new_instance)
        ):
            log(
                f"Stream is already running for device: {device} with correct settings, ignoring request",
                "DEBUG",
            )
            return

        # Remove the old instance if it exists
        if old_instance is not None:
            log(f"Stopping existing stream for device: {device}", "DEBUG")
            if old_instance.pipeline is not None:
                old_instance.pipeline.set_state(Gst.State.NULL)
            del gst_instances[device]

    # Create the elements
    source = _create_element("v4l2src", "source")
    caps = Gst.Caps.from_string(f"video/x-raw,width={new_instance.width},height={new_instance.height}")
    caps_filter = _create_element("capsfilter", "caps")
    convert = _create_element("videoconvert", "convert")
    flip = _create_element("videoflip", "flip")
    sink = _create_element("webrtcsink", "sink")
    # sink = _create_element("autovideosink", "sink")

    # Create the empty pipeline
    new_instance.pipeline = Gst.Pipeline.new(device)

    if not new_instance.pipeline or not source or not caps_filter or not convert or not flip or not sink:
        log("Not all elements could be created.", "ERROR")
        return

    # Build the pipeline. Note that we are NOT linking the source at this
    # point. We will do it later.
    new_instance.pipeline.add(source, caps_filter, convert, flip, sink)
    if not source.link(caps_filter) or not caps_filter.link(convert) or not convert.link(flip) or not flip.link(sink):
        log("Elements could not be linked.", "ERROR")
        return

    # Configure plugins
    source.set_property("device", f"{BY_ID_DIR}/{device}")
    caps_filter.set_property("caps", caps)
    flip.set_property("method", new_instance.transform)
    sink.set_property("stun-server", "NULL")
    # library typing is wrong we get a tuple here
    meta, _ = cast(tuple[Gst.Structure, None], Gst.Structure.from_string(f"meta,device={device}"))
    sink.set_property("meta", meta)
    signaller = sink.get_property("signaller")
    if signaller is None:
        log("Failed to get signaller from webrtcsink", "ERROR")
        return
    signaller.set_property("uri", f"ws://{web_server_ip}:8443")

    # Start playing
    ret = new_instance.pipeline.set_state(Gst.State.PLAYING)
    if ret == Gst.StateChangeReturn.FAILURE:
        log("Unable to set the pipeline to the playing state.", "ERROR")
        return

    # Listen to the bus
    bus = new_instance.pipeline.get_bus()
    bus.add_signal_watch()
    bus.connect(
        "message",
        _on_gst_message,
        device,
    )

    with gst_lock:
        gst_instances[device] = new_instance
