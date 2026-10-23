from dataclasses import dataclass, field
from typing import Optional, cast
import os

import gi
import threading

from server.message_types import CameraEventType, VideoTransformType, Device
from server.logger import log, log_level_type
from server.v4l_monitor import DEV_DIR

gi.require_version("GLib", "2.0")
gi.require_version("GObject", "2.0")
gi.require_version("Gst", "1.0")

# Formatter error E402 is silenced as the gi design pattern requires imports to be after the gi.require_version calls.
# gi.require_version("GLib", "2.0") generates typelib data at runtime.
# pylance throws an error (which we can ignore) on the following line due to this.
from gi.repository import Gst, GLib  # noqa: E402 # type: ignore[reportMissingImports]


@dataclass
class GstInstance:
    device: Device
    width: int
    height: int
    transform: VideoTransformType
    pipeline: Optional[Gst.Pipeline] = field(repr=False)
    redirect: Optional[Device] = None


Gst.init()

gst_lock = threading.Lock()
gst_instances: dict[str, GstInstance] = {}  # {videoXX: GstInstance}

glib_loop: GLib.MainLoop | None = None
glib_thread: threading.Thread | None = None

# How long we're willing to block waiting for a pipeline to actually
# finish tearing down before giving up and logging it instead of hanging
# forever. This blocks the GLib dispatch thread, so keep it short.
_TEARDOWN_TIMEOUT_NS = 2 * Gst.SECOND


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
        or old.redirect != new.redirect
    )


def _stop_pipeline(instance: GstInstance):
    """Actually tears a pipeline down: stop watching its bus, request NULL,
    and block (briefly) until that transition is confirmed, so we never
    discard a pipeline that's still mid-teardown. Must be called WITHOUT
    gst_lock held, since it can block for up to _TEARDOWN_TIMEOUT_NS."""
    pipeline = instance.pipeline
    if pipeline is None:
        return

    bus = pipeline.get_bus()
    bus.remove_signal_watch()  # stop dispatching messages for this pipeline

    pipeline.set_state(Gst.State.NULL)
    ret, state, _pending = pipeline.get_state(_TEARDOWN_TIMEOUT_NS)
    if ret != Gst.StateChangeReturn.SUCCESS or state != Gst.State.NULL:
        log(
            f"Pipeline for {instance.device} did not reach NULL cleanly "
            f"(ret={ret}, state={Gst.Element.state_get_name(state)})",
            "ERROR",
        )
    instance.pipeline = None


def _remove_and_stop(instance: GstInstance):
    """Removes `instance` from gst_instances (only if it's still the
    current entry for its device — guards against a stale/already-replaced
    instance's bus firing late) and tears its pipeline down."""
    with gst_lock:
        if gst_instances.get(instance.device.dev) is not instance:
            return
        del gst_instances[instance.device.dev]
    _stop_pipeline(instance)


def _on_gst_message(_, message: Gst.Message, instance: GstInstance):
    t = message.type
    type_name = Gst.MessageType.get_name(t)
    device = instance.device

    src = message.src
    src_name = src.get_name() if hasattr(src, "get_name") else None

    detail = ""
    log_level = "OK"
    try:
        match t:
            case Gst.MessageType.STATE_CHANGED:
                if src_name != device:
                    log_level = "DEBUG"
                old, new, pending = message.parse_state_changed()
                old_s = Gst.Element.state_get_name(old)
                new_s = Gst.Element.state_get_name(new)
                detail = f"{old_s} -> {new_s}"
                if pending != Gst.State.VOID_PENDING:
                    detail += f" (pending {Gst.Element.state_get_name(pending)})"

            case Gst.MessageType.STREAM_STATUS:
                status_type, owner = message.parse_stream_status()
                owner_name = (
                    owner.get_name() if hasattr(owner, "get_name") else str(owner)
                )
                detail = f"{status_type.value_nick} owner={owner_name}"

            case Gst.MessageType.NEW_CLOCK:
                clock = message.parse_new_clock()
                detail = f"clock={clock.get_name() if clock else 'None'}"

            case Gst.MessageType.STREAM_START:
                ok, group_id = message.parse_group_id()
                detail = f"group-id={group_id}" if ok else ""

            case Gst.MessageType.ASYNC_DONE:
                running_time = message.parse_async_done()
                detail = (
                    "running-time=none"
                    if running_time == Gst.CLOCK_TIME_NONE
                    else f"running-time={running_time}"
                )

            case Gst.MessageType.LATENCY:
                detail = "recalculate latency"

            case Gst.MessageType.NEED_CONTEXT:
                _, context_type = message.parse_context_type()
                detail = f"context-type={context_type}"

            case Gst.MessageType.HAVE_CONTEXT:
                context = message.parse_have_context()
                detail = f"context-type={context.get_context_type()}"

            case Gst.MessageType.TAG:
                taglist = message.parse_tag()
                detail = taglist.to_string()

            case Gst.MessageType.ERROR | Gst.MessageType.WARNING | Gst.MessageType.INFO:
                parse_fn = {
                    Gst.MessageType.ERROR: message.parse_error,
                    Gst.MessageType.WARNING: message.parse_warning,
                    Gst.MessageType.INFO: message.parse_info,
                }[t]
                log_level = cast(
                    log_level_type,
                    {
                        Gst.MessageType.ERROR: "ERROR",
                        Gst.MessageType.WARNING: "WARN",
                        Gst.MessageType.INFO: "OK",
                    }[t],
                )
                gerror, debug = parse_fn()
                detail = f"{gerror.message}" + (f" | {debug}" if debug else "")

                if t == Gst.MessageType.ERROR:
                    detail += " | tearing down pipeline"
                    _remove_and_stop(instance)

            case Gst.MessageType.EOS:
                log_level = "WARN"
                detail = "Freeing pipeline resources"
                _remove_and_stop(instance)

            case _:
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
        redirect=event.data.redirect if event.data else None,
        pipeline=None,
    )
    force_restart = (
        event.data.forceRestart if event.data and event.data.forceRestart else False
    )

    old_instance: Optional[GstInstance] = None
    with gst_lock:
        existing = gst_instances.get(device.dev, None)
        if (
            existing is not None
            and not force_restart
            and not _instance_changed(existing, new_instance)
        ):
            log(
                f"Stream is already running for device: {device} with correct settings, ignoring request",
                "DEBUG",
            )
            return
        log(
            f"Request stream for device: {event.target} ({new_instance.width}x{new_instance.height}, transform={new_instance.transform}, force_restart={force_restart})"
        )
        if existing is not None:
            del gst_instances[device.dev]
            old_instance = existing

    if old_instance is not None:
        log(f"Stopping existing stream for device: {device}", "DEBUG")
        _stop_pipeline(old_instance)

    # Create the elements
    source = _create_element("v4l2src", "source")
    convert = _create_element("videoconvert", "convert")
    scale = _create_element("videoscale", "scale")
    caps = Gst.Caps.from_string(
        f"video/x-raw,width={new_instance.width},height={new_instance.height}"
    )
    caps_filter = _create_element("capsfilter", "caps")
    flip = _create_element("videoflip", "flip")
    tee = _create_element("tee", "tee")
    v4l2_queue = _create_element("queue", "v4l2_queue")
    webrtc_queue = _create_element("queue", "webrtc_queue")
    v4l2_sink = _create_element("v4l2sink", "v4l2_sink")
    webrtc_sink = _create_element("webrtcsink", "webrtc_sink")

    new_instance.pipeline = Gst.Pipeline.new(device.dev)

    if (
        not new_instance.pipeline
        or not source
        or not convert
        or not scale
        or not caps_filter
        or not flip
        or not tee
        or not v4l2_queue
        or not webrtc_queue
        or not v4l2_sink
        or not webrtc_sink
    ):
        log("Not all elements could be created.", "ERROR")
        return

    # Build the pipeline. Note that we are NOT linking the source at this
    # point. We will do it later.
    new_instance.pipeline.add(source, convert, caps_filter, scale, flip, webrtc_sink)
    # pipeline: source -> convert -> scale -> caps_filter -> flip -> ...
    if (
        not source.link(convert)
        or not convert.link(scale)
        or not scale.link(caps_filter)
        or not caps_filter.link(flip)
    ):
        log("Elements could not be linked.", "ERROR")
        return

    source.set_property("device", os.path.join(DEV_DIR, device.dev))
    caps_filter.set_property("caps", caps)
    flip.set_property("method", new_instance.transform)

    if new_instance.redirect is not None:
        new_instance.pipeline.add(tee, v4l2_queue, v4l2_sink, webrtc_queue)
        if (
            not flip.link(tee)
            or not tee.link(v4l2_queue)
            or not v4l2_queue.link(v4l2_sink)
            or not tee.link(webrtc_queue)
            or not webrtc_queue.link(webrtc_sink)
        ):
            log("Elements could not be linked.", "ERROR")
            return

        v4l2_sink.set_property(
            "device", os.path.join(DEV_DIR, new_instance.redirect.dev)
        )

    else:
        if not flip.link(webrtc_sink):
            log("Elements could not be linked.", "ERROR")
            return

    webrtc_sink.set_property("stun-server", "NULL")
    # library typing is wrong we get a tuple here
    meta, _ = cast(
        tuple[Gst.Structure, None],
        Gst.Structure.from_string(f"meta,device={device.hostname}:{device.dev}"),
    )
    log(f"meta,device={device.hostname}_{device.dev}")
    webrtc_sink.set_property("meta", meta)
    signaller = webrtc_sink.get_property("signaller")
    if signaller is None:
        log("Failed to get signaller from webrtcsink", "ERROR")
        return
    signaller.set_property("uri", f"ws://{web_server_ip}:8443")

    ret = new_instance.pipeline.set_state(Gst.State.PLAYING)
    if ret == Gst.StateChangeReturn.FAILURE:
        log("Unable to set the pipeline to the playing state.", "ERROR")
        _stop_pipeline(new_instance)  # clean up what we just built
        return

    bus = new_instance.pipeline.get_bus()
    bus.add_signal_watch()
    bus.connect(
        "message", _on_gst_message, new_instance
    )  # pass the instance, not a string

    with gst_lock:
        gst_instances[device.dev] = new_instance


def stop_stream(event: CameraEventType):
    device = event.target if event.target else None
    if device is None:
        log("No device specified in event", "ERROR")
        return

    with gst_lock:
        instance = gst_instances.get(device.dev, None)
        if instance is None:
            log(f"No stream running for device: {device}", "DEBUG")
            return
        del gst_instances[device.dev]

    log(f"Stopping stream for device: {device}")
    _stop_pipeline(instance)


def stop_all_streams():
    with gst_lock:
        instances = list(gst_instances.values())
        gst_instances.clear()

    for instance in instances:
        _stop_pipeline(instance)
