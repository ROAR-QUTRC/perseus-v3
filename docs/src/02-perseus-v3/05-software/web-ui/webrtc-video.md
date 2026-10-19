# WebRTC Video

## Camera Server

The function of the camera server is to:

- Provide an interface to connected video devices
- Generate Gstreamer pipelines and manage instances
- Communicate with the main web server

This server has been implemented with python and can be found in the `software/native/camera_server` directory. This directory also include a `config.json` file, which is used to set the server name and defaults for the cli.

## Usage

### CLI

The server is packaged om the `webui` devenv profile as `camera_server`. The CLI takes two optional arguments, `--debug` which enables debug logging and `hostname:port`.

The second argument is used to set the hostname and port of the web server which the camera server will connect to. If this argument is not provided the server will default to `localhost:3000` (this is the production port for the Web UI when hosted on the same machine). The `hostname` or the `:port` can be omitted, in which case the default will be used. 

### Config

!!! failure "Please do not commit local changes to the `config.json` file."

The `config.json` file is used to set the default hostname and port for the web server. The file is located in the same directory as the camera server and has the following format and defaults:

```json
{
  "name": "pi-brain",
  "hostname": "localhost",
  "port": 5173
}
```

!!! warning "In all cases the CLI arguments will take priority."

## Development Details

### Video Device Detection

Detecting and filtering video devices is done through `pyudev` and the `/sys/class/video4linux` managed by the v4l2 driver. `pyudev` provides an event callback which is triggered whenever a device is connected or disconnected. The camera server will then determine if the device is a usb or virtual video capable device and otherwise ignore the event. The video devices are then uniquely identified using the device ID (`videoXX`) and the camera server name. This is a compromise made to allow for multiple identical devices to be streamed from. 

??? info "Duplicate Device Problems"

    Ideally, each camera device would have a unique identifier as part of the name that is symlinked in `/dev/v4l/by-id/`. However, this is not always the case with cheap PCB cameras which instead leave a symlink to the most recently connected device. In short we must rely on the `videoXX` server name combination because:

    - `/dev/v4l/by-id/` only shows unique entries for cameras with unique identifiers, which is not always the case
    - Creating symlinks in `/dev/v4l/by-id/` is problematic as it requires sudo and must be done on every boot
    - The `videoXX` ID on it's own is not enough since we need to use multiple devices to distribute load

### Gstreamer Pipelines

The server uses the `Gst` and `gi` python libraries to generate and manage Gstreamer pipelines. Each pipeline is generated based on the requested device, resolution and transform. The server will also monitor the pipeline for errors and restart it if necessary. The plugins used are:

| Plugin | Purpose |
|--------|---------|
| [v4l2src](https://gstreamer.freedesktop.org/documentation/video4linux2/v4l2src.html) | Provides access to video devices |
| [videoconvert](https://gstreamer.freedesktop.org/documentation/videoconvertscale/videoconvert.html) | Converts video formats | 
| [videoscale](https://gstreamer.freedesktop.org/documentation/videoconvertscale/videoscale.html) | Scales video dimensions. This is done with a plugin and not `videoconvert` so it scales the frame and doesnt request a differnt resolution from the cameras. Resizing like this allows all the cameras to stream at their native resolution and all resolutions requested will result in an image. |
| `video/x-raw,width=WIDTH,height=HEIGHT` | Sets the resolution of the video stream |
| [videoflip](https://gstreamer.freedesktop.org/documentation/videofilter/videoflip.html) | Flips the video stream based on the requested transform |
| [tee](https://gstreamer.freedesktop.org/documentation/coreelements/tee.html) | Forks the pipeline to two sinks, this is used when a virtual device redirect is required. |
| [queue](https://gstreamer.freedesktop.org/documentation/coreelements/queue.html) | Buffers the video stream to prevent pipeline stalls for both the original and redirected streams but only needed when redirecting the stream. |
| [v4l2sink](https://gstreamer.freedesktop.org/documentation/video4linux2/v4l2sink.html) | Redirects the video stream to a virtual device. The camera server does not handle creating virtual devices, this must be done by the user as it requires sudo. |
| [webrtcbin](https://gstreamer.freedesktop.org/documentation/rswebrtc/webrtcsink.html) | Handles the WebRTC connection and streaming to the web server. Note that the `stun-server` is disabled as this is only intended for local networks and the signalling server is run by the main Web UI server. |


!!! tip "Permission Denied Error"

    To fix a permission denied error when trying to connect to a camera use:

    ```console
    sudo chmod 777 /dev/video*
    ```

    This gives read/write/execute permissions to all video devices.

### WebSocket Communication Protocol

The communication with the web server is conducted via a WebSocket with the goal of sending information about the available devices to the client and then initialising requested streams. All messages are sent using the following interface.

```ts
interface CameraEventType {
    type: 'camera';
    action:
        | 'group-description'
        | 'kill-stream'
        | 'request-groups'
        | 'request-stream'
        | 'group-terminated'
        | 'device-connect'
        | 'device-disconnect';
    target?: DeviceType;
    devices?: Array<DeviceType>;
    data: {
        resolution?: { width: number; height: number };
        transform?: VideoTransformType;
        forceRestart?: boolean;
        redirect?: DeviceType;
    };
}
```

<!-- Will add a data flow diagram here once the drawio PR is merged -->
