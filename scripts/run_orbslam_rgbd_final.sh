#!/usr/bin/env bash
# Run ONE ORB-SLAM3 RGB-D pass over a compressed Orbbec bag with a GRACEFUL
# shutdown, so the node's destructor runs SaveKeyFrameTrajectoryTUM +
# SaveTrajectoryTUM (the per-frame FINAL, loop-closed trajectory -- see
# patches/apply_patch.py) in addition to recording the causal /slam/pose
# stream for comparison.
#
# Mirrors output_fr3/run_graceful.sh (cd into the writable output dir before
# launching rgbd_node_cpp, then SIGINT -- never SIGKILL -- the node after
# playback ends and wait for it to exit) combined with run_orbslam_rgbd.sh's
# Orbbec decompress + causal-pose-record + eval pipeline.
#
# Usage: run_orbslam_rgbd_final.sh BAG CONFIG_YAML OUTDIR LABEL [RATE]
#   OUTDIR is also used as the SLAM node's CWD, so CameraTrajectory.txt and
#   KeyFrameTrajectory.txt land there directly.
set -eo pipefail
BAG="$1"; CONFIG="$2"; OUTDIR="$3"; LABEL="${4:-run}"; RATE="${5:-1.0}"

source /opt/ros/jazzy/setup.bash
source /opt/orbslam3_ws/install/setup.bash
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
: "${ROS_DOMAIN_ID:=0}"
export ROS_DOMAIN_ID
VOC=/opt/orbslam3_ws/src/ros2_orb_slam3/orb_slam3/Vocabulary/ORBvoc.txt.bin
CDIR="$(cd "$(dirname "$CONFIG")" && pwd)/"
CNAME="$(basename "$CONFIG" .yaml)"
RGBD=/opt/orbslam3_ws/install/lib/ros2_orb_slam3/rgbd_node_cpp
POSE_BAG="${OUTDIR}/${LABEL}_poses"
mkdir -p "$OUTDIR"; rm -rf "$POSE_BAG"

echo "[run] LABEL=$LABEL CONFIG=$CONFIG RATE=$RATE DOMAIN=$ROS_DOMAIN_ID"
now() { date +%s.%N; }
T_start=$(now)

# 1. decompress compressed color/depth -> raw Image for the rgbd node
python3 "$(dirname "$0")/decompress_rgbd_node.py" --ros-args \
  -p color_in:=/camera/color/image_raw/compressed \
  -p depth_in:=/camera/depth/image_raw/compressed \
  -p color_out:=/camera/color/image_raw \
  -p depth_out:=/camera/depth/image_raw \
  -p color_encoding:=bgr8 -p sync:="${DECOMP_SYNC:-true}" > "${OUTDIR}/${LABEL}.decompress.log" 2>&1 &
DPID=$!

# 2. ORB-SLAM3 RGB-D node, CWD = OUTDIR so CameraTrajectory.txt /
#    KeyFrameTrajectory.txt land there at shutdown.
(
  cd "$OUTDIR"
  exec "$RGBD" --ros-args \
    -p voc_file_arg:="$VOC" \
    -p settings_file_path_arg:="$CDIR" \
    -p settings_name_arg:="$CNAME" \
    -p color_topic:=/camera/color/image_raw \
    -p depth_topic:=/camera/depth/image_raw > "${OUTDIR}/${LABEL}.slam.log" 2>&1
) &
SPID=$!
SLAM_LOG="${OUTDIR}/${LABEL}.slam.log"
for i in $(seq 1 120); do
  grep -qiE "node ready" "$SLAM_LOG" 2>/dev/null && break
  sleep 0.25
done
sleep 1
T_ready=$(now)
echo "[run] node ready pid=$SPID"

# 3. record causal estimate + reference
ros2 bag record -s mcap -o "${OUTDIR}/${LABEL}_poses" /slam/pose /camera_pose > "${OUTDIR}/${LABEL}.record.log" 2>&1 &
RPID=$!
sleep 2

# 4. play the bag (blocks until end)
T_play0=$(now)
ros2 bag play "$BAG" -r "$RATE" > "${OUTDIR}/${LABEL}.play.log" 2>&1
T_play1=$(now)
echo "[run] playback finished"
sleep 4

# 5. graceful shutdown: SIGINT the recorder, then SIGINT the SLAM node and
#    WAIT for it to exit on its own (its destructor writes the trajectory
#    files) -- never SIGKILL it during normal operation.
kill -INT "$RPID" 2>/dev/null || true; sleep 3
echo "[run] sending SIGINT to SLAM node $SPID"
kill -INT "$SPID" 2>/dev/null || true
for i in $(seq 1 60); do
  kill -0 "$SPID" 2>/dev/null || { echo "[run] node exited after ${i}s"; break; }
  sleep 1
done
if kill -0 "$SPID" 2>/dev/null; then
  echo "[run] WARNING: node still alive after 60s, forcing"
  kill -9 "$SPID" || true
fi
kill -INT "$DPID" 2>/dev/null || true
kill "$DPID" 2>/dev/null || true
wait 2>/dev/null || true
T_eval0=$(now)

# 6. evaluate causal trajectory vs O3D reference (bag-embedded /camera_pose)
python3 "$(dirname "$0")/eval_traj.py" "$POSE_BAG" --label "${LABEL}_causal" --json "${OUTDIR}/${LABEL}_causal.eval.json" || true
T_eval1=$(now)

ls -la "$OUTDIR"
awk -v a="$T_start" -v b="$T_ready" -v c="$T_play0" -v d="$T_play1" -v lbl="$LABEL" 'BEGIN{
  printf "[timing] %s init=%.1fs play=%.1fs\n", lbl, b-a, d-c
}'
echo "DONE"
