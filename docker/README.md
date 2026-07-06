# Docker

Reference image for `ros_rtsp` on ROS Noetic. See
[`docs/IMPLEMENTATION.md` §11](../docs/IMPLEMENTATION.md#11-deployment) for the full
explanation.

Build (context = repo root):

```bash
docker build -t ros_rtsp -f docker/Dockerfile .
```

Run — **CPU / x264**:

```bash
docker run --rm -it --network host \
  --device /dev/video0 \
  -v $PWD/config/stream_setup.yaml:/root/catkin_ws/src/ros_rtsp/config/stream_setup.yaml \
  ros_rtsp
```

Run — **NVENC / GPU** (needs the NVIDIA Container Toolkit on the host, and
`encoder: nvenc` on the streams you want hardware-encoded):

```bash
docker run --rm -it --network host --gpus all \
  -e NVIDIA_DRIVER_CAPABILITIES=compute,video,utility \
  -v $PWD/config/stream_setup.yaml:/root/catkin_ws/src/ros_rtsp/config/stream_setup.yaml \
  ros_rtsp
```

Notes:

- `--network host` avoids per-port UDP forwarding for the negotiated RTP/RTCP
  streams.
- `--device /dev/video0` is only needed for `cam` / `v4l2src` streams.
- Mounting your own `stream_setup.yaml` reconfigures without rebuilding.
- The image starts its own `roscore` for a self-contained demo; set
  `-e ROS_MASTER_URI=http://<master>:11311` (with `--network host`) to attach to an
  existing master instead. Verify NVENC inside the container with
  `gst-inspect-1.0 nvh264enc`.

> The Dockerfile is a tested-by-inspection reference (it was not built inside the
> authoring environment). Adjust the base image/tags to your platform as needed.
