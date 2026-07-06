# Reference image for ros_rtsp on ROS Noetic (Ubuntu 20.04).
#
# CPU (x264) build/run:
#   docker build -t ros_rtsp -f docker/Dockerfile .
#   docker run --rm -it --network host \
#     -v $PWD/config/stream_setup.yaml:/root/catkin_ws/src/ros_rtsp/config/stream_setup.yaml \
#     ros_rtsp
#
# NVENC (GPU) run — same image, hardware enabled by the NVIDIA Container Toolkit:
#   docker run --rm -it --network host --gpus all \
#     -e NVIDIA_DRIVER_CAPABILITIES=compute,video,utility \
#     -v $PWD/config/stream_setup.yaml:/root/catkin_ws/src/ros_rtsp/config/stream_setup.yaml \
#     ros_rtsp
#   (set `encoder: nvenc` on the streams you want on the GPU)
#
# --network host is recommended: RTSP negotiates UDP RTP/RTCP ports that are
# awkward to forward one by one. Add `--device /dev/video0` for v4l2 `cam` streams.
FROM ros:noetic-ros-base

# GStreamer: core + base plugins (appsrc/videoconvert/videoscale) + rtsp-server,
# plus the good/bad/ugly plugin sets (bad ships nvh264enc/nvcodec, ugly ships x264enc).
RUN apt-get update && apt-get install -y --no-install-recommends \
      libgstreamer1.0-dev \
      libgstreamer-plugins-base1.0-dev \
      libgstrtspserver-1.0-dev \
      gstreamer1.0-plugins-good \
      gstreamer1.0-plugins-bad \
      gstreamer1.0-plugins-ugly \
    && rm -rf /var/lib/apt/lists/*

ENV CATKIN_WS=/root/catkin_ws
WORKDIR ${CATKIN_WS}

# Copy the package into the workspace and resolve any remaining rosdeps.
COPY . ${CATKIN_WS}/src/ros_rtsp
RUN apt-get update && \
    rosdep update --rosdistro noetic && \
    rosdep install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

# Build the workspace.
RUN . /opt/ros/noetic/setup.sh && catkin_make -DCMAKE_BUILD_TYPE=Release

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# RTSP control port (RTP/RTCP use additional negotiated UDP ports — use host net).
EXPOSE 8554

ENTRYPOINT ["/entrypoint.sh"]
CMD ["roslaunch", "ros_rtsp", "rtsp_streams.launch"]
