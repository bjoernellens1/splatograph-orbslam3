#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash
source /opt/orbslam3_ws/install/setup.bash
ros2 pkg prefix ros2_orb_slam3 >/dev/null
test -x /opt/orbslam3_ws/install/lib/ros2_orb_slam3/mono_node_cpp
command -v slam-launch >/dev/null
