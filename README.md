# ros_rtsp
ROS package to subscribe to an ROS Image topic (and as many other video sources as you want) and serve it up as a RTSP video feed with different mount points.
Should provide a real-time video feed (or as close as possible).

Runs on Ubuntu 16.04 / 18.04 / 20.04 with ROS kinetic, melodic and **noetic**.

> **Docs:** see [`docs/IMPLEMENTATION.md`](docs/IMPLEMENTATION.md) for full
> implementation documentation (architecture, code walkthrough, configuration,
> encoder/NVENC internals, build system and deployment), and
> [`docker/`](docker/) for a ready-to-build Docker image (CPU and NVENC/GPU).


## Dependencies
- ROS

- gstreamer libs. Either let `rosdep` pull them in (they are declared in `package.xml`):
```bash
rosdep install --from-paths . --ignore-src -r -y
```
or install them manually:
```bash
sudo apt-get install libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstreamer-plugins-good1.0-dev libgstreamer-plugins-bad1.0-dev libgstrtspserver-1.0-dev gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly
```

## Build into your catkin workspace
Navigate to your catkin workspace `src` folder. I.e. `cd ~/catkin_ws/src`.
Clone this package to the repository:
```bash
git clone https://github.com/CircusMonkey/ros_rtsp.git
```

Navigate back to the catkin workspace root and make the package:
```bash
cd ..
catkin_make pkg:=ros_rtsp
source devel/setup.bash
```

## Quick test (no camera needed)
To verify the install end-to-end without any camera or ROS Image publisher, launch
the built-in example. It serves a GStreamer test pattern over RTSP:
```bash
roslaunch ros_rtsp example.launch
```
Then play it (see [Checking the streams](#checking-the-streams) for client commands):
```
rtsp://127.0.0.1:8554/test     # moving ball
rtsp://127.0.0.1:8554/clock    # SMPTE bars + clock overlay (handy for latency)
```

## Stream Setup
Change the `config/stream_setup.yaml` to suit your required streams.

### Just point at a ROS topic
For a ROS Image topic, only `type` and `source` are required — everything else has
a sensible default:

```yaml
streams:
  backcam:                       # the stream name; also the default mountpoint
    type: topic
    source: /camera/image_raw    # the sensor_msgs/Image topic to serve
    #                            -> rtsp://<server_ip>:8554/backcam
```

Defaults when a field is omitted: `mountpoint` → `/<stream name>`, `bitrate` → `500`
kbit/s, `encoder` → `x264`, and with no `caps` the topic is served at its **native
resolution**.

### Full options
```yaml
streams:
  # ROS Image topic, every optional knob spelled out
  frontcam:
    type: topic
    source: /usb_cam0/image_raw
    mountpoint: /front     # RTSP path (default: /<stream name>)
    bitrate: 800           # H.264 target, kbit/s (default: 500)
    encoder: nvenc         # x264 (default, software) or nvenc (NVIDIA hardware). See below.
    caps: video/x-raw,framerate=10/1,width=640,height=480  # optional: rescale / cap framerate

  # A non-ROS GStreamer source (e.g. a v4l2 camera) served directly
  usbcam:
    type: cam              # cam - not sourced from ROS; 'source' is a full GStreamer source ending in raw video
    source: "v4l2src device=/dev/video0 ! videoconvert ! videoscale ! video/x-raw,framerate=15/1,width=1280,height=720"
    mountpoint: /usb
    bitrate: 800
```
Add as many streams as you require.

## Hardware acceleration (NVENC)
Encoding is the most CPU-hungry part of the pipeline. Each stream can pick its H.264 encoder with the optional `encoder` parameter:

- `encoder: x264` &nbsp;— software encoding (default). Works everywhere, no GPU required.
- `encoder: nvenc` — NVIDIA hardware encoding (`nvh264enc`). Offloads encoding to the GPU for much lower CPU usage and latency.

`bitrate` is in kbit/sec for both and needs no change when switching. NVENC needs the proprietary NVIDIA driver and the `nvcodec` GStreamer plugin (shipped in `gstreamer1.0-plugins-bad`). Verify it is available with:
```bash
gst-inspect-1.0 nvh264enc
```

For any other hardware encoder (Intel/AMD VA-API, Jetson `nvv4l2h264enc`, ...) set `encoder_override` to the full encoder + output-caps fragment and it is used verbatim:
```yaml
  # Intel / AMD VA-API
  encoder_override: "videoconvert ! vaapih264enc rate-control=cbr bitrate=500 keyframe-period=30 ! video/x-h264, profile=baseline"

  # NVIDIA Jetson (bitrate here is in bits/sec)
  encoder_override: "nvvidconv ! nvv4l2h264enc bitrate=500000 insert-sps-pps=true iframeinterval=30 maxperf-enable=1 ! video/x-h264, profile=baseline"
```

## Checking the streams
Launch the streams from the built-in ROS launch file (it loads
`config/stream_setup.yaml`):
```bash
roslaunch ros_rtsp rtsp_streams.launch
```

The launch file takes optional arguments:
```bash
# Use your own config without editing the package:
roslaunch ros_rtsp rtsp_streams.launch config:=/abs/path/to/my_streams.yaml

# Load into an existing nodelet manager (e.g. your camera driver's) for
# zero-copy image transport instead of starting a standalone one:
roslaunch ros_rtsp rtsp_streams.launch start_manager:=false manager:=<existing_manager>
```

| arg | default | purpose |
| --- | ------- | ------- |
| `config` | `$(find ros_rtsp)/config/stream_setup.yaml` | stream configuration file to load |
| `manager` | `rtsp_nodelet_manager` | nodelet manager name to load the RTSP nodelet into |
| `start_manager` | `true` | start the manager (`false` to reuse an existing one) |

In the following examples, replace the `rtsp://127.0.0.1:8554/front` with your servers IP address and mount point `rtsp://YOUR_IP:8554/MOUNT_POINT`.

### gstreamer
Use `gst-launch-1.0`. You will need to install gstreamer for your client system. See https://gstreamer.freedesktop.org/documentation/installing/index.html
```bash
gst-launch-1.0 -v rtspsrc location=rtsp://127.0.0.1:8554/front drop-on-latency=true use-pipeline-clock=true do-retransmission=false latency=0 protocols=GST_RTSP_LOWER_TRANS_UDP ! rtph264depay ! h264parse ! avdec_h264 ! autovideosink sync=true
```

### mpv
```bash
mpv --no-cache --untimed --no-demuxer-thread --vd-lavc-threads=1 rtsp://127.0.0.1:8554/front
```

### VLC
VLC adds way too much latency. Please don't use it for this purpose. If you want to try, this is the command that was the least slow (Let me know if you find a better command):
```bash
cvlc --no-audio --mux none --demux none --deinterlace 0 --no-autoscale --avcodec-hw=any --no-auto-preparse --sout-rtp-proto=udp --network-caching=300 --realrtsp-caching=0 --sout-udp-caching=0 --clock-jitter=0 --rtp-max-misorder=0 rtsp://127.0.0.1:8554/front :udp-timeout=0
```
If you wish to use the VLC mobile app to stream on Android or iOS, navigate to the network or streams menu and type in your server URL and mountpoint e.g. `rtsp://127.0.0.1:8554/front`


## Debugging
- If too much latency is encounted with multiple streams running, the server computer may be maxing out its processor trying to encode all the streams. Try reducing the resolution of the source caps, or switch that stream to `encoder: nvenc`.
- If too many frames are being dropped, it is likely due to network bandwidth. Try dropping the bitrate.

### Reading the node's startup log
The node logs exactly what it did on startup; match the line you see to the cause:

| Log line | Meaning / fix |
| -------- | ------------- |
| `Stream '<name>' available at rtsp://0.0.0.0:8554/<mount>` | That mount registered fine. |
| `image2rtsp: N of M stream(s) registered on port 8554.` | Summary. If `N < M`, the skipped streams logged a reason just above. |
| `No valid 'streams' parameter under namespace '/...'` | The YAML didn't load into the node — check the launch file's `rosparam load` and the namespace it names. |
| `Stream 'x' … has no 'source' …` / `unknown type …` | That stream's config is incomplete; fix the named field. Other streams still run. |
| `Stream 'x' skipped - bad parameter …` | A field has the wrong type (e.g. `bitrate: "500"` as a string is tolerated, but truly non-numeric isn't). |
| `source topic '/foo' is not currently advertised` | The topic isn't being published yet. The stream is still registered; it works once something publishes to `/foo` (check the name/namespace if it never does). |
| `Stream /cam: no publisher on source topic '/foo'` (recurring) | A client is connected but nothing is publishing to the source — clients get no video until it does. |
| `Stream /cam: receiving frames from '/foo' (WxH enc)` | Frames are flowing — the stream is live. |
| `GST: image encoding '<enc>' is not supported` | The topic publishes an encoding this node can't map (use `rgb8`/`bgr8`/`mono8`/…). |
| `Failed to start the RTSP server on port 8554 …` | Port already in use — another instance/RTSP server is still running. |

### Client-side symptoms
| Symptom | Cause / fix |
| ------- | ----------- |
| `no factory for path /foo` | No mount is registered at `/foo`. Check the startup log for a `Stream … available at …/foo` line; if it's missing, that stream didn't register (see the table above). Make sure the mount path matches exactly (leading `/`). |
| `could not prepare media` / 503 after a delay | The mount exists but no frames are flowing — the source topic isn't publishing (see "no publisher" above), or publishes an unsupported encoding. |
| Stream connects but is laggy | Use the `gst-launch`/`mpv` client commands below, not VLC (VLC buffers heavily). Reduce `bitrate` if the network is the bottleneck. |
| `assertion 'path[0] == '/'' failed` (older builds) | A mountpoint without a leading `/`. Current builds normalise this automatically; rebuild, or add the leading `/`. |

### Stale build / library shadowing
If code changes seem to have no effect, another package may be shadowing the nodelet library, or `devel/` is stale. The library is `libros_rtsp_nodelet.so`; make sure only one copy is on the path:
```bash
find / -name 'libros_rtsp_nodelet.so' -o -name 'libimage_to_rtsp_nodelet.so' 2>/dev/null   # expect one, from your workspace
cd ~/catkin_ws && rm -rf build devel && catkin_make && source devel/setup.bash             # clean rebuild
```
