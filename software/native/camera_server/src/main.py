import sys
import socketio
import gi

from logger import log, enable_debug

gi.require_version("GLib", "2.0")
gi.require_version("GObject", "2.0")
gi.require_version("Gst", "1.0")

# Formatter error E402 is silenced as the gi design pattern requires imports to be after the gi.require_version calls.
# gi.require_version("GLib", "2.0") generates typelib data at runtime.
# pylance throws an error (which we can ignore) on the following line due to this.

# sio = socketio.AsyncServer(async_mode="asgi", cors_allowed_origins="*")

# videoTransformType = Literal[
#     "none",
#     "clockwise",
#     "counterclockwise",
#     "rotate-180",
#     "horizontal-flip",
#     "vertical-flip",
#     "upper-left-diagonal",
#     "upper-right-diagonal",
#     "automatic",
# ]


# pipeline = None
# bus = None
# message = None

# # initialize GStreamer
# Gst.init(sys.argv[1:])

# # build the pipeline
# pipeline = Gst.parse_launch(
#     "playbin uri=https://gstreamer.freedesktop.org/data/media/sintel_trailer-480p.webm"
# )

# # start playing
# pipeline.set_state(Gst.State.PLAYING)

# # wait until EOS or error
# bus = pipeline.get_bus()
# msg = bus.timed_pop_filtered(
#     Gst.CLOCK_TIME_NONE, Gst.MessageType.ERROR | Gst.MessageType.EOS
# )

# # free resources
# pipeline.set_state(Gst.State.NULL)


def main():
    args = sys.argv[1:]
    if "--debug" in args:
        enable_debug()
        args.remove("--debug")
    log("Debug logging enabled", "DEBUG")

    hostname = "localhost"
    port = 3000
    if len(sys.argv) > 0:
        # hostname:port passed as command line argument
        arg = sys.argv[1]
        if ":" in arg:
            hostname, port_str = arg.split(":", 1)
            try:
                port = int(port_str)
            except ValueError:
                log(f"Invalid port number: {port_str}", "ERROR")
                sys.exit(1)
            if hostname == "":
                hostname = "localhost"
        else:
            hostname = arg
    log(f"Connecting to webserver at {hostname}:{port}")

    # Connect to web server socket
    with socketio.SimpleClient() as sio:
        try:
            sio.connect(f"http://{hostname}:{port}")
            log("Connected to web server socket")
        except Exception as e:
            log(f"Failed to connect to web server socket: {e}", "ERROR")
            sys.exit(1)


if __name__ == "__main__":
    main()
