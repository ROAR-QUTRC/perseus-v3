To record data, use this command:

```bash
gst-launch-1.0 v4l2src ! video/x-raw,width=640,height=480 ! videorate ! video/x-raw,framerate=2/1 ! videoconvert ! jpegenc ! multifilesink location=~/database/%05d.jpg
```
