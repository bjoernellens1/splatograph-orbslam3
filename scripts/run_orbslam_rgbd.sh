#!/usr/bin/env bash
# Run ONE ORB-SLAM3 RGB-D pass over an Orbbec bag and evaluate the trajectory
# against the baked-in /camera_pose (O3D) reference.
#
# Inside the :jazzy-orbbec-v3 image. Mounts expected:
#   /scripts      -> this repo's scripts/   (decompress + eval)
#   /splatograph  -> splatograph repo       (utils/trajectory_eval.py, PYTHONPATH)
#   <bag/out>     -> a writable dir for the recorded poses bag + json
#
# Usage: run_orbslam_rgbd.sh BAG CONFIG_YAML OUTDIR LABEL [RATE] [DURATION_S]
#   DURATION_S (optional): cap playback to this many bag-seconds (smoke/iteration).
set -eo pipefail
BAG="$1"; CONFIG="$2"; OUTDIR="$3"; LABEL="${4:-run}"; RATE="${5:-0.5}"; DURATION="${6:-0}"
PLAY_EXTRA=""
[ "$DURATION" != "0" ] && PLAY_EXTRA="--playback-duration $DURATION"

source /opt/ros/jazzy/setup.bash
source /opt/orbslam3_ws/install/setup.bash
export PYTHONPATH="/splatograph:${PYTHONPATH:-}"
VOC=/opt/orbslam3_ws/src/ros2_orb_slam3/orb_slam3/Vocabulary/ORBvoc.txt.bin
CDIR="$(cd "$(dirname "$CONFIG")" && pwd)/"
CNAME="$(basename "$CONFIG" .yaml)"
RGBD=/opt/orbslam3_ws/install/lib/ros2_orb_slam3/rgbd_node_cpp
POSE_BAG="${OUTDIR}/${LABEL}_poses"
mkdir -p "$OUTDIR"; rm -rf "$POSE_BAG"

echo "[run] LABEL=$LABEL CONFIG=$CONFIG RATE=$RATE DOMAIN=$ROS_DOMAIN_ID"

# 1. decompress compressed color/depth -> raw Image for the rgbd node
python3 /scripts/decompress_rgbd_node.py --ros-args \
  -p color_in:=/camera/color/image_raw/compressed \
  -p depth_in:=/camera/depth/image_raw/compressed \
  -p color_out:=/camera/color/image_raw \
  -p depth_out:=/camera/depth/image_raw \
  -p color_encoding:=bgr8 > "${OUTDIR}/${LABEL}.decompress.log" 2>&1 &
DPID=$!

# 2. ORB-SLAM3 RGB-D node (loads vocab; give it time)
"$RGBD" --ros-args \
  -p voc_file_arg:="$VOC" \
  -p settings_file_path_arg:="$CDIR" \
  -p settings_name_arg:="$CNAME" \
  -p color_topic:=/camera/color/image_raw \
  -p depth_topic:=/camera/depth/image_raw > "${OUTDIR}/${LABEL}.slam.log" 2>&1 &
SPID=$!
sleep 15

# 3. record estimate + reference
ros2 bag record -s mcap -o "$POSE_BAG" /slam/pose /camera_pose > "${OUTDIR}/${LABEL}.record.log" 2>&1 &
RPID=$!
sleep 3

# 4. play the bag (blocks until end)
ros2 bag play "$BAG" -r "$RATE" $PLAY_EXTRA > "${OUTDIR}/${LABEL}.play.log" 2>&1
sleep 4

# 5. shut down recorder + slam + decompress
kill -INT "$RPID" 2>/dev/null || true; sleep 3
kill -INT "$SPID" "$DPID" 2>/dev/null || true; sleep 2
kill "$SPID" "$DPID" "$RPID" 2>/dev/null || true
wait 2>/dev/null || true

# 6. evaluate trajectory vs O3D reference
python3 /scripts/eval_traj.py "$POSE_BAG" --label "$LABEL" --json "${OUTDIR}/${LABEL}.eval.json"
