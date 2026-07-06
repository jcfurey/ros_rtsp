# `ros_rtsp` — Implementation Documentation

This document describes **how `ros_rtsp` works internally**, how it is built, how it
is configured, and how to deploy it (native and Docker, with and without an NVIDIA
GPU). It targets ROS 1 **Noetic** on Ubuntu 20.04 but also runs on Melodic (18.04)
and Kinetic (16.04).

- [1. Overview](#1-overview)
- [2. Architecture](#2-architecture)
- [3. Runtime data flow](#3-runtime-data-flow)
- [4. Package layout](#4-package-layout)
- [5. Code walkthrough](#5-code-walkthrough)
- [6. Configuration reference](#6-configuration-reference)
- [7. Encoder subsystem (x264 / NVENC / override)](#7-encoder-subsystem-x264--nvenc--override)
- [8. GStreamer pipeline anatomy](#8-gstreamer-pipeline-anatomy)
- [9. Build system](#9-build-system)
- [10. Dependencies](#10-dependencies)
- [11. Deployment](#11-deployment)
- [12. Client playback](#12-client-playback)
- [13. Performance tuning](#13-performance-tuning)
- [14. Troubleshooting](#14-troubleshooting)
- [15. Extending the package](#15-extending-the-package)
- [16. Known limitations](#16-known-limitations)
- [17. References](#17-references)

---

## 1. Overview

`ros_rtsp` is a ROS 1 **nodelet** that publishes one or more **RTSP** H.264 video
streams. Each stream has its own RTSP *mount point* (e.g. `rtsp://host:8554/front`)
and comes from one of two sources:

| Stream `type` | Source | Encoded from |
| ------------- | ------ | ------------ |
| `topic` | a `sensor_msgs/Image` ROS topic | raw frames pushed from ROS into GStreamer via `appsrc` |
| `cam`   | any GStreamer source string (e.g. `v4l2src`) | frames produced entirely inside GStreamer, ROS never sees them |

The heavy lifting (H.264 encoding, RTP packetisation, the RTSP protocol, session
management, multicast) is done by **GStreamer** and its
**`gst-rtsp-server`** library. The nodelet is essentially glue that:

1. reads the stream definitions from the ROS parameter server,
2. builds a GStreamer *launch string* per stream and registers it with the RTSP
   server as a *media factory*,
3. for `topic` streams, subscribes to the ROS image topic **only while a client is
   connected** and forwards each frame into the running GStreamer pipeline.

The design goal is **low latency** ("as close to real-time as possible"), so the
pipelines are tuned for zero-latency encoding and no buffering.

---

## 2. Architecture

```
                                   ROS parameter server
                                   (config/stream_setup.yaml)
                                            │  streams:, port:
                                            ▼
┌──────────────────────────────────────────────────────────────────────────┐
│                     Image2RTSPNodelet  (one process)                       │
│                                                                            │
│   ROS spinner thread                    GLib main-loop thread              │
│   ───────────────────                   ─────────────────────              │
│   onInit(): parse params,               g_main_loop_run() on the           │
│     build pipelines,                     default GMainContext:             │
│     create RTSP server        ┌───────►   • RTSP protocol I/O              │
│                               │           • client-connected signal        │
│   imageCallback():            │           • session cleanup timer (2 s)    │
│     sensor_msgs/Image  ──push─┘           • media-configure signal         │
│         │        (thread-safe GstAppSrc)                                   │
│         ▼                                                                   │
│   ┌───────────────── GStreamer pipeline (per stream) ─────────────────┐    │
│   │ appsrc ─► videoconvert ─►[videoscale ─► caps]─► ENCODER ─► h264parse   │
│   │        (or v4l2src ... for a `cam` stream)  ^optional      │        │  │
│   │                                                     ▼        ▼         │
│   │                                          x264enc / nvh264enc  rtph264pay (pay0) │
│   └────────────────────────────────────────────────────────────────────┘    │
└──────────────────────────────────────────────────────────────────────────┘
                                            │ RTSP (TCP 8554) + RTP/RTCP (UDP)
                                            ▼
                              RTSP clients (gstreamer, mpv, VLC, ...)
```

Two threads matter:

- **ROS callback thread** — runs `imageCallback()` for each incoming
  `sensor_msgs/Image` and pushes the frame into the pipeline's `appsrc`.
  `GstAppSrc` push is thread-safe, so no explicit locking is needed.
- **GLib main-loop thread** — created in `video_mainloop_start()`. It drives the
  RTSP server (protocol handling, the `client-connected` signal, the periodic
  session-cleanup timer, and the `media-configure` callback that wires up each new
  pipeline instance).

`gst-rtsp-server` builds a **fresh pipeline instance the first time a client
requests a mount point**. Because every factory is created with
`gst_rtsp_media_factory_set_shared(factory, TRUE)`, all clients of the same mount
point share **one** pipeline (one encoder), which is what keeps CPU/GPU cost
independent of the number of viewers.

---

## 3. Runtime data flow

### 3.1 `topic` stream (ROS Image → RTSP)

```
sensor_msgs/Image topic
      │  (subscribed only while ≥1 client is connected)
      ▼
imageCallback(msg, topic)
      │  gst_caps_new_from_image(msg)   → set caps on appsrc
      │  gst_buffer_new_allocate + fill → wrap msg->data
      ▼
gst_app_src_push_buffer(appsrc)
      ▼
appsrc ! videoconvert ! [videoscale ! <caps> !] <encoder> ! h264parse ! rtph264pay
      ▼                    ^ only when the stream sets `caps`; omitted = native resolution
RTP over UDP to the client
```

The subscription is **lazy**: `url_connected()` subscribes on the first client and
`url_disconnected()` unsubscribes when the last client leaves. With no viewers, the
node consumes no encoding CPU and does not even touch the image topic.

### 3.2 `cam` stream (GStreamer-only)

```
v4l2src (or any GStreamer source) ! ... ! <encoder> ! h264parse ! rtph264pay
      ▼
RTP over UDP to the client
```

ROS is not involved in the data path at all; the source is a raw-video GStreamer
element string supplied in the config. `cam` streams also get a **multicast
address pool** (see [5.7](#57-media_configure--appsrc-wiring--multicast-pool)).

---

## 4. Package layout

```
ros_rtsp/
├── CMakeLists.txt            # catkin + pkg-config build
├── package.xml               # ROS package manifest / dependencies
├── nodelet_plugins.xml       # nodelet plugin export descriptor
├── include/
│   └── image2rtsp.h          # Image2RTSPNodelet class declaration
├── src/
│   ├── image2rtsp.cpp        # nodelet: params, pipeline strings, encoder, ROS callbacks
│   └── video.cpp             # GStreamer/RTSP server glue, main loop, signals
├── config/
│   ├── stream_setup.yaml     # your stream definitions (edit this)
│   └── example_stream_setup.yaml  # demo streams used by example.launch
├── launch/
│   ├── rtsp_streams.launch   # built-in launch (args: config, manager, start_manager)
│   └── example.launch        # self-contained demo (GStreamer test pattern, no camera)
└── docs/
    └── IMPLEMENTATION.md      # this file
```

### Why a nodelet (not a node)?

The class derives from `nodelet::Nodelet`. Nodelets load into a **nodelet manager**
process and can share a process (and zero-copy intraprocess transport) with the
image publisher. For a `topic` stream fed by a camera driver that is also a
nodelet, this avoids serialising/copying every frame over the ROS transport. The
provided launch file uses a *standalone* manager, but you can load `ros_rtsp` into
the **same** manager as your camera nodelet to get the zero-copy benefit.

---

## 5. Code walkthrough

### 5.1 `onInit()` — nodelet entry point (`src/image2rtsp.cpp`)

`onInit()` is called by the nodelet manager after construction. It:

1. Sets `GST_DEBUG=*:1` (errors only) if the user hasn't set it, keeping GStreamer
   quiet by default.
2. Reads parameters from the **private** node handle:
   - `port` (string, e.g. `"8554"`),
   - `streams` (an `XmlRpc::XmlRpcValue` struct of stream definitions).
3. Starts the GLib main loop (`video_mainloop_start()`) and creates the RTSP server
   (`rtsp_server_create(port)`).
4. Iterates over every stream, and for each one:
   - reads `mountpoint` and `bitrate`,
   - calls `build_encoder()` to get the encoder fragment,
   - assembles the full GStreamer launch string,
   - registers it with the RTSP server via `rtsp_server_add_url()`.

The **shared tail** appended to every pipeline is:

```
 ! h264parse ! rtph264pay name=pay0 pt=96 config-interval=1 )
```

- `h264parse` normalises the H.264 stream (alignment/format) for the payloader; it
  is a no-op passthrough for `x264enc` output and improves robustness for hardware
  encoders.
- `rtph264pay name=pay0` — `gst-rtsp-server` requires the RTP payloader for stream
  *N* to be named `payN`; `pay0` marks this as the first (only) stream.
- `config-interval=1` re-sends **SPS/PPS** (the H.264 parameter sets) once per
  second in-band. Without it, a client that connects *after* the first keyframe may
  show a black/stalled picture until the next parameter set arrives. Because
  factories are `shared`, mid-stream joins are the common case, so this matters.

### 5.2 `build_encoder()` — encoder selection (`src/image2rtsp.cpp`)

Returns the pipeline fragment from the **encoder element through its output caps**
(everything between the raw-video head and `h264parse`). Selection logic:

1. If the stream defines **`encoder_override`**, return it verbatim (full escape
   hatch for any custom encoder).
2. Otherwise read **`encoder`** (default `"x264"`), lower-case it, and map:
   - `nvenc` / `nvh264enc` / `nv` →
     `videoconvert ! nvh264enc bitrate=<b> gop-size=30 rc-mode=cbr preset=low-latency-hq ! video/x-h264, profile=baseline`
   - `x264` / `x264enc` / `sw` (and the default) →
     `x264enc tune=zerolatency bitrate=<b> key-int-max=30 ! video/x-h264, profile=baseline`
   - anything else → warn and fall back to x264.

`bitrate` is passed through unchanged; both `x264enc` and `nvh264enc` interpret it
as **kbit/sec**, so switching encoders needs no other change. See
[section 7](#7-encoder-subsystem-x264--nvenc--override) for the deep dive.

### 5.3 `video_mainloop_start()` — GLib thread (`src/video.cpp`)

```c
gst_init(NULL, NULL);                      // initialise GStreamer once
pthread_create(&tloop, NULL, &mainloop, NULL);
```

`mainloop()` creates a `GMainLoop` on the **default** `GMainContext` and runs it
forever. All `gst-rtsp-server` sources (the listening socket, timeouts, signals)
are attached to that default context by `gst_rtsp_server_attach(server, NULL)`, so
they are serviced on this thread.

### 5.4 `rtsp_server_create()` — server setup (`src/video.cpp`)

```c
server = gst_rtsp_server_new();
g_object_set(server, "service", port.c_str(), NULL);   // TCP port, e.g. "8554"
gst_rtsp_server_attach(server, NULL);                  // attach to default context
g_signal_connect(server, "client-connected", G_CALLBACK(new_client), this);
g_timeout_add_seconds(2, (GSourceFunc)session_cleanup, this);  // reap dead sessions
```

`session_cleanup()` runs every 2 s and calls
`gst_rtsp_session_pool_cleanup()` to expire timed-out RTSP sessions (important for
UDP clients that vanish without a clean `TEARDOWN`).

### 5.5 Client connect / disconnect → lazy ROS subscription

`new_client()` connects two per-client signals:

- `options-request` → `client_options()` → `nodelet->url_connected(uri->abspath)`
- `teardown-request` → `client_teardown()` → `nodelet->url_disconnected(uri->abspath)`

`url_connected(url)` (in `image2rtsp.cpp`) walks the `streams` param, and for the
`topic` stream whose `mountpoint` equals the requested URL path:

```cpp
if (num_of_clients[url] == 0)
    subs[url] = nh.subscribe<sensor_msgs::Image>(
        source, 1, boost::bind(&Image2RTSPNodelet::imageCallback, this,
                               boost::placeholders::_1, url));
num_of_clients[url]++;
```

- Queue size `1` keeps only the newest frame (drops stale frames → lower latency).
- `boost::bind(..., url)` binds the mount point so the callback knows which
  `appsrc` to feed.

`url_disconnected(url)` decrements the client count and, when it hits zero,
`subs[url].shutdown()` (stop consuming the topic) and clears `appsrc[url]`.

> Note: connection is detected on the RTSP `OPTIONS` request. Most players send
> `OPTIONS` at the start of every session, which is why it is used as the
> "connected" trigger.

### 5.6 `imageCallback()` — the ROS→GStreamer bridge

```cpp
if (appsrc[topic] != NULL) {
    caps = gst_caps_new_from_image(msg);       // caps from this frame
    gst_app_src_set_caps(appsrc[topic], caps);
    buf = gst_buffer_new_allocate(nullptr, msg->data.size(), nullptr);
    gst_buffer_fill(buf, 0, msg->data.data(), msg->data.size());
    GST_BUFFER_FLAG_SET(buf, GST_BUFFER_FLAG_LIVE);
    gst_app_src_push_buffer(appsrc[topic], buf);  // hand to the pipeline
}
```

`appsrc[topic]` is only non-NULL after `media_configure()` has wired it up (i.e.
after a client connected and the pipeline was instantiated), so frames are dropped
until there is a live pipeline to receive them. Buffers are timestamped by the
pipeline because `appsrc` is configured with `do-timestamp=true` and `format=time`.

### 5.7 `media_configure()` — appsrc wiring & multicast pool (`src/video.cpp`)

Fired (on the GLib thread) whenever a new pipeline instance is constructed for a
mount point. The `appsrc` argument distinguishes the two stream kinds:

- **`topic` stream** (`appsrc != NULL`): find the `appsrc name=imagesrc` element in
  the freshly built pipeline and store its pointer into the nodelet's `appsrc`
  map, and set `format=time`. From now on `imageCallback()` can push into it.
- **`cam` stream** (`appsrc == NULL`): assign each stream a **multicast address
  pool** (`224.3.0.x`, ports `5000+`). This lets `cam` streams be delivered by
  multicast when a client requests it.

### 5.8 `gst_caps_new_from_image()` — encoding → GStreamer format map

Maps `sensor_msgs/image_encodings` (RGB8, BGR8, MONO8, …) to GStreamer raw video
formats (RGB, BGR, GRAY8, …), rejects big-endian images, and builds
`video/x-raw` caps with the image `width`/`height`. (The framerate in these caps is
a fixed `10/1`; see [Known limitations](#16-known-limitations).)

### 5.9 `rtsp_server_add_url()` — register a mount point (`src/video.cpp`)

```c
mounts  = gst_rtsp_server_get_mount_points(rtsp_server);
factory = gst_rtsp_media_factory_new();
gst_rtsp_media_factory_set_launch(factory, sPipeline);       // the launch string
g_signal_connect(factory, "media-configure", (GCallback)media_configure, appsrc);
gst_rtsp_media_factory_set_shared(factory, TRUE);            // one pipeline, many clients
gst_rtsp_mount_points_add_factory(mounts, url, factory);     // bind to /mountpoint
```

The factory lazily parses `sPipeline` (gst-launch syntax) the **first time** a
client requests the URL. A syntactically invalid pipeline (e.g. `encoder: nvenc`
on a host without the NVIDIA plugin) does **not** crash the node at startup — the
client instead gets an RTSP "could not prepare media" error, and the rest of the
streams keep working.

---

## 6. Configuration reference

Configuration is a YAML file loaded onto the private parameter namespace by the
launch file (`config/stream_setup.yaml`).

### Top level

| Key | Type | Default | Meaning |
| --- | ---- | ------- | ------- |
| `port` | string | `"8554"` | TCP port the RTSP server listens on. |
| `streams` | map | — | Map of stream definitions (the map keys are arbitrary names). |

### Per-stream keys

| Key | Applies to | Required | Meaning |
| --- | ---------- | -------- | ------- |
| `type` | all | ✅ | `topic` or `cam`. |
| `source` | all | ✅ | `topic`: the `sensor_msgs/Image` topic. `cam`: a GStreamer source string ending in raw video. |
| `mountpoint` | all | ❌ (default `/<stream name>`) | RTSP path, e.g. `/front` → `rtsp://host:8554/front`. |
| `bitrate` | all | ❌ (default `500`) | Target H.264 bitrate in **kbit/sec** (for `x264`/`nvenc`). |
| `caps` | `topic` | ❌ (default: native resolution) | When set, inserts `videoscale ! <caps>` to rescale / cap the framerate before the encoder. Omit to serve the topic as-is. |
| `encoder` | all | ❌ (default `x264`) | `x264` (software) or `nvenc` (NVIDIA hardware). |
| `encoder_override` | all | ❌ | Full custom `encoder … ! caps` fragment; overrides `encoder`. |

So the minimal `topic` stream is just `type` + `source`; the map key doubles as the
default mountpoint.

### Example

```yaml
port: "8554"
streams:
  # Minimal: point at a topic, take every default (served at native resolution
  # on /backcam, x264 @ 500 kbit/s)
  backcam:
    type: topic
    source: /camera/image_raw

  front-cam:
    type: cam
    source: "v4l2src device=/dev/video0 ! videoconvert ! videoscale ! video/x-raw,framerate=15/1,width=1280,height=720"
    mountpoint: /front
    bitrate: 800
    encoder: x264

  robot-camera:
    type: topic
    source: /usb_cam0/image_raw
    mountpoint: /back
    caps: video/x-raw,framerate=10/1,width=640,height=480
    bitrate: 500
    encoder: nvenc          # GPU H.264

  intel-igpu:
    type: topic
    source: /front_cam/image_raw
    mountpoint: /vaapi
    bitrate: 1000
    encoder_override: "videoconvert ! vaapih264enc rate-control=cbr bitrate=1000 keyframe-period=30 ! video/x-h264, profile=baseline"
```

---

## 7. Encoder subsystem (x264 / NVENC / override)

Encoding is the most CPU-intensive stage, so the encoder is selectable per stream.

### 7.1 `x264` — software (default)

```
x264enc tune=zerolatency bitrate=<kbps> key-int-max=30 ! video/x-h264, profile=baseline
```

| Property | Purpose |
| -------- | ------- |
| `tune=zerolatency` | disables B-frames and lookahead → minimal encode latency |
| `bitrate` | target in **kbit/sec** |
| `key-int-max=30` | force a keyframe at least every 30 frames (bounds join latency) |
| `profile=baseline` (caps) | maximal client compatibility; no B-frames |

Works on any machine with no special hardware. Highest CPU cost; the practical
limiter for many simultaneous / high-resolution streams.

### 7.2 `nvenc` — NVIDIA hardware

```
videoconvert ! nvh264enc bitrate=<kbps> gop-size=30 rc-mode=cbr preset=low-latency-hq ! video/x-h264, profile=baseline
```

| Property | Purpose |
| -------- | ------- |
| `bitrate` | target in **kbit/sec** (same units as x264 — no config change needed) |
| `gop-size=30` | keyframe interval (analogue of `key-int-max`) |
| `rc-mode=cbr` | constant bitrate — predictable for streaming |
| `preset=low-latency-hq` | NVENC low-latency high-quality preset |
| leading `videoconvert` | guarantees a pixel format NVENC accepts (NV12/I420/…) |

Encoding runs on the GPU's dedicated NVENC block, so it barely touches the CPU and
scales to many/large streams. `nvh264enc` in this configuration accepts
**system-memory** `video/x-raw` and uploads internally, so no explicit CUDA/GL
memory handling is required in the pipeline.

**Requirements:** the proprietary NVIDIA driver plus the GStreamer **`nvcodec`**
plugin, which ships inside `gstreamer1.0-plugins-bad`. The plugin only registers
`nvh264enc` when the NVIDIA userspace libraries (`libnvidia-encode.so`,
`libcuda.so`) are visible at load time. Verify with:

```bash
gst-inspect-1.0 nvh264enc     # prints the element's properties if available
```

### 7.3 `encoder_override` — anything else

Set the whole encoder-through-caps fragment yourself. Used verbatim, so you own the
element names, properties and units. Ready-made examples:

```yaml
# Intel / AMD VA-API (gstreamer1.0-vaapi). bitrate in kbit/sec.
encoder_override: "videoconvert ! vaapih264enc rate-control=cbr bitrate=500 keyframe-period=30 ! video/x-h264, profile=baseline"

# NVIDIA Jetson (L4T gstreamer). bitrate here is in BITS/sec (500000 = 500 kbit/s).
encoder_override: "nvvidconv ! nvv4l2h264enc bitrate=500000 insert-sps-pps=true iframeinterval=30 maxperf-enable=1 ! video/x-h264, profile=baseline"

# Software HEVC (x265) if you need better compression than baseline H.264.
encoder_override: "x265enc tune=zerolatency bitrate=500 key-int-max=30 ! video/x-h265"
```

> The shared tail is `h264parse ! rtph264pay`. If you override with an **H.265**
> encoder you must adapt the tail as well (use `h265parse ! rtph265pay`); the
> current build hardcodes the H.264 payloader, so H.265 requires a code change to
> the tail in `onInit()`.

---

## 8. GStreamer pipeline anatomy

The exact launch strings the node builds (with the example config values):

**`topic` + `x264`, no `caps` (minimal — native resolution)**
```
( appsrc name=imagesrc do-timestamp=true min-latency=0 max-latency=0 max-bytes=1000 is-live=true
  ! videoconvert
  ! x264enc tune=zerolatency bitrate=500 key-int-max=30 ! video/x-h264, profile=baseline
  ! h264parse ! rtph264pay name=pay0 pt=96 config-interval=1 )
```

**`topic` + `x264` with `caps` (rescale / cap framerate)**
```
( appsrc name=imagesrc do-timestamp=true min-latency=0 max-latency=0 max-bytes=1000 is-live=true
  ! videoconvert ! videoscale ! video/x-raw,framerate=10/1,width=640,height=480
  ! x264enc tune=zerolatency bitrate=500 key-int-max=30 ! video/x-h264, profile=baseline
  ! h264parse ! rtph264pay name=pay0 pt=96 config-interval=1 )
```

**`topic` + `nvenc` with `caps`**
```
( appsrc name=imagesrc do-timestamp=true min-latency=0 max-latency=0 max-bytes=1000 is-live=true
  ! videoconvert ! videoscale ! video/x-raw,framerate=10/1,width=640,height=480
  ! videoconvert ! nvh264enc bitrate=500 gop-size=30 rc-mode=cbr preset=low-latency-hq ! video/x-h264, profile=baseline
  ! h264parse ! rtph264pay name=pay0 pt=96 config-interval=1 )
```

**`cam` + `x264`**
```
( v4l2src device=/dev/video0 ! videoconvert ! videoscale ! video/x-raw,framerate=15/1,width=1280,height=720
  ! x264enc tune=zerolatency bitrate=500 key-int-max=30 ! video/x-h264, profile=baseline
  ! h264parse ! rtph264pay name=pay0 pt=96 config-interval=1 )
```

### Why the `appsrc` properties matter

| Property | Effect |
| -------- | ------ |
| `is-live=true` | source is live; disables prerolling, keeps latency bounded |
| `do-timestamp=true` | GStreamer stamps buffers on arrival (we don't rely on ROS header stamps) |
| `min-latency=0 max-latency=0` | no added buffering latency |
| `max-bytes=1000` | tiny internal queue — apply backpressure / drop rather than accumulate lag |

---

## 9. Build system

### `CMakeLists.txt`

- `cmake_minimum_required(VERSION 3.0.2)` — matches the catkin era and avoids
  deprecation errors on modern CMake.
- `CMAKE_CXX_STANDARD 14` — Noetic's toolchain and modern GStreamer/Boost headers
  expect C++14.
- GStreamer libraries are located with **pkg-config**, not catkin:
  ```cmake
  find_package(PkgConfig REQUIRED)
  pkg_check_modules(GST REQUIRED gstreamer-1.0 gstreamer-app-1.0 gstreamer-rtsp-server-1.0)
  ```
  `${GST_INCLUDE_DIRS}` / `${GST_LIBRARIES}` are then added to the target.
- The nodelet is a **shared library** target `image_to_rtsp_nodelet`
  (built from `src/image2rtsp.cpp` + `src/video.cpp`) linked against
  `${catkin_LIBRARIES}` and `${GST_LIBRARIES}`.

> There is **no** `find_package(OpenCV)`: the code does not use OpenCV, so the
> dependency was removed to keep the build minimal.

### `package.xml`

- Format 2 manifest.
- ROS deps: `roscpp`, `sensor_msgs`, `nodelet`.
- GStreamer deps declared so `rosdep` resolves them on Noetic/20.04:
  `libgstreamer1.0-dev`, `libgstreamer-plugins-base1.0-dev`,
  `libgstrtspserver-1.0-dev` (build+run), and the `good`/`bad`/`ugly` plugin
  packages (runtime). `nvh264enc` lives in `gstreamer1.0-plugins-bad`.
- Exports the nodelet plugin:
  ```xml
  <export><nodelet plugin="${prefix}/nodelet_plugins.xml" /></export>
  ```

### `nodelet_plugins.xml`

Declares the class `image2rtsp/Image2RTSPNodelet`
(type `image2rtsp::Image2RTSPNodelet`, base `nodelet::Nodelet`) in the library
`lib/libimage_to_rtsp_nodelet`, so `pluginlib` can load it by name.

### Building

```bash
cd ~/catkin_ws/src
git clone https://github.com/jcfurey/ros_rtsp.git
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y    # pull GStreamer deps
catkin_make            # or: catkin build ros_rtsp
source devel/setup.bash
```

---

## 10. Dependencies

| Component | Package (Ubuntu 20.04 / Noetic) | rosdep key |
| --------- | ------------------------------- | ---------- |
| GStreamer core (dev) | `libgstreamer1.0-dev` | `libgstreamer1.0-dev` |
| Base plugins (dev; `appsrc`, `videoconvert`, `videoscale`) | `libgstreamer-plugins-base1.0-dev` | `libgstreamer-plugins-base1.0-dev` |
| RTSP server (dev) | `libgstrtspserver-1.0-dev` | `libgstrtspserver-1.0-dev` |
| Good plugins (runtime; RTP payloaders, `v4l2src`) | `gstreamer1.0-plugins-good` | `gstreamer1.0-plugins-good` |
| Bad plugins (runtime; **`nvh264enc`/nvcodec**, `h264parse`) | `gstreamer1.0-plugins-bad` | `gstreamer1.0-plugins-bad` |
| Ugly plugins (runtime; `x264enc`) | `gstreamer1.0-plugins-ugly` | `gstreamer1.0-plugins-ugly` |
| ROS | `roscpp`, `sensor_msgs`, `nodelet` | (catkin) |
| NVENC only | proprietary NVIDIA driver | (installed separately) |
| VA-API only | `gstreamer1.0-vaapi` | — |

Manual install:

```bash
sudo apt-get install \
  libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev libgstrtspserver-1.0-dev \
  gstreamer1.0-plugins-good gstreamer1.0-plugins-bad gstreamer1.0-plugins-ugly
```

---

## 11. Deployment

### 11.1 Native (catkin)

Quick self-contained demo (no camera or Image publisher needed — serves a
GStreamer test pattern):

```bash
roslaunch ros_rtsp example.launch
# then play rtsp://127.0.0.1:8554/test  (or /clock)
```

Normal launch (loads `config/stream_setup.yaml`):

```bash
roslaunch ros_rtsp rtsp_streams.launch
# streams announced on stdout, e.g.:
#   Stream available at rtsp://0.0.0.0:8554/front
```

`rtsp_streams.launch` arguments:

| arg | default | purpose |
| --- | ------- | ------- |
| `config` | `$(find ros_rtsp)/config/stream_setup.yaml` | stream configuration file to load |
| `manager` | `standalone_nodelet` | nodelet manager to load the RTSP nodelet into |
| `start_manager` | `true` | start the manager, or `false` to reuse an existing one |

```bash
# own config:
roslaunch ros_rtsp rtsp_streams.launch config:=/abs/path/to/my_streams.yaml
```

To get **zero-copy** from a camera nodelet, load `ros_rtsp` into the *same* nodelet
manager as the camera driver instead of starting a standalone one:

```bash
roslaunch ros_rtsp rtsp_streams.launch start_manager:=false manager:=<existing_manager>
```

`example.launch` is just a thin wrapper that `include`s `rtsp_streams.launch` with
`config:=$(find ros_rtsp)/config/example_stream_setup.yaml`.

### 11.2 Docker — CPU (x264)

A reference image is provided in [`docker/Dockerfile`](../docker/Dockerfile). Build
and run it on the host network (RTSP negotiates UDP ports, so host networking is by
far the simplest):

```bash
docker build -t ros_rtsp -f docker/Dockerfile .

docker run --rm -it \
  --network host \
  --device /dev/video0 \            # only needed for `cam`/v4l2 streams
  -v $PWD/config/stream_setup.yaml:/root/catkin_ws/src/ros_rtsp/config/stream_setup.yaml \
  ros_rtsp
```

Mounting your own `stream_setup.yaml` lets you reconfigure without rebuilding.

### 11.3 Docker — NVENC (GPU)

Same image. NVENC is enabled **entirely at run time** by the NVIDIA Container
Toolkit — the `nvcodec` plugin (already in `gstreamer1.0-plugins-bad`) picks up the
driver libraries that the toolkit injects. You must:

1. Install the [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
   on the host.
2. Pass `--gpus all` **and** request the `video` driver capability (the capability
   that mounts `libnvidia-encode.so`; it is *not* implied by `all` on every setup,
   so set it explicitly):

```bash
docker run --rm -it \
  --network host \
  --gpus all \
  -e NVIDIA_DRIVER_CAPABILITIES=compute,video,utility \
  -v $PWD/config/stream_setup.yaml:/root/catkin_ws/src/ros_rtsp/config/stream_setup.yaml \
  ros_rtsp
```

Set `encoder: nvenc` on the streams you want hardware-encoded. Verify inside the
container with `gst-inspect-1.0 nvh264enc`.

---

## 12. Client playback

Replace `127.0.0.1` with the server IP and `/front` with your mount point.

**GStreamer (lowest latency):**
```bash
gst-launch-1.0 -v rtspsrc location=rtsp://127.0.0.1:8554/front \
  drop-on-latency=true use-pipeline-clock=true do-retransmission=false \
  latency=0 protocols=GST_RTSP_LOWER_TRANS_UDP \
  ! rtph264depay ! h264parse ! avdec_h264 ! autovideosink sync=true
```

**mpv:**
```bash
mpv --no-cache --untimed --no-demuxer-thread --vd-lavc-threads=1 rtsp://127.0.0.1:8554/front
```

**VLC** adds significant latency and is not recommended for real-time use; if you
must, see the tuned `cvlc` command in the top-level `README.md`.

---

## 13. Performance tuning

- **Move encoding to hardware.** `encoder: nvenc` (or a VA-API override) frees the
  CPU and is the single biggest win for many/large streams.
- **Right-size resolution and framerate** in `caps` / the `cam` source. Encoding
  cost scales with pixels × fps. `10/1` fps is a good default for `topic` streams.
- **Match bitrate to the network.** Dropped frames on the client usually mean the
  bitrate exceeds available bandwidth; lower `bitrate`.
- **Keyframe interval vs. join latency.** Smaller `key-int-max`/`gop-size` lets new
  clients start faster but costs bitrate. `config-interval=1` already mitigates the
  parameter-set part of join latency.
- **Prefer UDP** on the client (`protocols=...UDP`, `latency=0`) for low latency;
  fall back to TCP interleaving only across lossy/NAT'd links.
- **Co-locate nodelets** (load into the camera's nodelet manager) to avoid copying
  frames for `topic` streams.

---

## 14. Troubleshooting

| Symptom | Likely cause / fix |
| ------- | ------------------ |
| `could not prepare media` after a delay when a client connects | The pipeline failed to build: the source topic isn't publishing yet, a `cam` device is missing, or the chosen encoder element isn't installed. Re-run with `GST_DEBUG=2` (or higher) to see the parse/link error. |
| `gst-inspect-1.0 nvh264enc` prints nothing / `encoder: nvenc` fails | NVIDIA driver or `nvcodec` plugin not visible. Install the driver + `gstreamer1.0-plugins-bad`; in Docker add `--gpus all -e NVIDIA_DRIVER_CAPABILITIES=compute,video,utility`. |
| Black screen / no picture until much later | Client connected mid-stream and missed SPS/PPS. `config-interval=1` should fix it; ensure your build includes it and that the client requests a keyframe. |
| High latency with several streams | CPU saturated by `x264enc`. Switch to `nvenc`/VA-API, or reduce resolution/fps. |
| Choppy / dropped frames | Network bandwidth exceeded → lower `bitrate`; or the `topic` publisher is slower than the caps framerate. |
| `topic` stream never shows anything | No client is connected yet (subscription is lazy), the `source` topic name is wrong, or the image `encoding`/endianness is unsupported (see `gst_caps_new_from_image`). |
| Build can't find GStreamer | Install the `-dev` packages / run `rosdep install` (section 10). |

Increase GStreamer logging for a single run:

```bash
GST_DEBUG=3 roslaunch ros_rtsp rtsp_streams.launch
```

---

## 15. Extending the package

- **Add a named encoder.** Add a branch to `build_encoder()` returning the
  `encoder … ! caps` fragment. Keep `bitrate` in kbit/sec, or convert inside the
  branch if the element uses different units (e.g. Jetson `nvv4l2h264enc` uses
  bits/sec).
- **Add H.265 (HEVC).** Change the shared tail in `onInit()` to
  `h265parse ! rtph265pay name=pay0 pt=96 config-interval=1` and provide an H.265
  encoder fragment. Clients must support HEVC-over-RTSP.
- **Per-stream framerate for `topic` streams.** Make the fixed `10/1` in
  `gst_caps_new_from_image()` configurable (read it from the stream params).
- **Authentication / TLS.** `gst-rtsp-server` supports `GstRTSPAuth` and TLS; wire
  it up in `rtsp_server_create()`.

---

## 16. Known limitations

- **Fixed appsrc framerate.** `gst_caps_new_from_image()` always advertises
  `framerate=10/1` on the `appsrc` caps regardless of the actual publish rate (buffers
  are still timestamped by arrival via `do-timestamp=true`). Set a stream's `caps` with
  an explicit `framerate=` to override it, or make the default configurable (section 15).
- **H.264 only in the built-in tail.** The shared tail hardcodes
  `h264parse ! rtph264pay`. Non-H.264 encoders via `encoder_override` require a
  matching code change to the tail.
- **Connection detected via `OPTIONS`.** Clients that don't send an `OPTIONS`
  request won't trigger the lazy subscription; virtually all standard players do.
- **No authentication by default.** Anyone who can reach the port can view the
  streams.
- **`bitrate` units differ across override encoders.** kbit/sec for
  `x264enc`/`nvh264enc`/`vaapih264enc`; bits/sec for Jetson `nvv4l2h264enc` and
  `omxh264enc`. The override string is used verbatim, so set the right units.

---

## 17. References

- GStreamer `gst-rtsp-server` — RTSP server library
  <https://gstreamer.freedesktop.org/documentation/gst-rtsp-server/>
- GStreamer `nvh264enc` (nvcodec) —
  <https://gstreamer.freedesktop.org/documentation/nvcodec/nvh264enc.html>
- GStreamer `x264enc` —
  <https://gstreamer.freedesktop.org/documentation/x264/index.html>
- GStreamer `rtph264pay` (`config-interval`) —
  <https://gstreamer.freedesktop.org/documentation/rtp/rtph264pay.html>
- GStreamer `appsrc` —
  <https://gstreamer.freedesktop.org/documentation/app/appsrc.html>
- NVIDIA Container Toolkit (`NVIDIA_DRIVER_CAPABILITIES`, `--gpus`) —
  <https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/docker-specialized.html>
- ROS nodelet —
  <http://wiki.ros.org/nodelet>
- Original appsrc/caps approach adapted from ProjectArtemis `gst_video_server` —
  <https://github.com/ProjectArtemis/gst_video_server>
