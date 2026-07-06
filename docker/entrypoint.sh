#!/usr/bin/env bash
# Source ROS and the built workspace, then exec the command (default: roslaunch).
set -e

source /opt/ros/noetic/setup.bash
source "${CATKIN_WS:-/root/catkin_ws}/devel/setup.bash"

# Start a roscore in the background if one isn't already reachable, so the image
# is self-contained for a quick demo. Point ROS_MASTER_URI at an external master
# to use your own roscore instead.
if ! rostopic list >/dev/null 2>&1; then
  roscore &
  until rostopic list >/dev/null 2>&1; do sleep 0.2; done
fi

exec "$@"
