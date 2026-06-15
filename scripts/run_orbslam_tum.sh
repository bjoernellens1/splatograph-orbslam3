#!/usr/bin/env bash
# Run ONE ORB-SLAM3 RGB-D pass over a ros2-converted TUM bag and evaluate the
# trajectory against the TUM mocap groundtruth.txt (Umeyama-aligned ATE).
#
# Unlike run_orbslam_rgbd.sh (Orbbec), TUM bags are ALREADY raw sensor_msgs/Image
# (/camera/rgb/image_color rgb8, /camera/depth/image 32FC1 metres) -- so there is
# NO decompress node, and the reference is groundtruth.txt, not a baked
# /camera_pose. Records to sqlite3; explicit-PID shutdown.
#
# Usage: run_orbslam_tum.sh BAG CONFIG_YAML GT_TXT OUTDIR LABEL [RATE]
set -eo pipefail
BAG="$1"; CONFIG="$2"; GT="$3"; OUTDIR="$4"; LABEL="${5:-tum}"; RATE="${6:-1.0}"
source /opt/ros/jazzy/setup.bash
source /opt/orbslam3_ws/install/setup.bash
export PYTHONPATH="/splatograph:${PYTHONPATH:-}"
VOC=/opt/orbslam3_ws/src/ros2_orb_slam3/orb_slam3/Vocabulary/ORBvoc.txt.bin
CDIR="$(cd "$(dirname "$CONFIG")" && pwd)/"
CNAME="$(basename "$CONFIG" .yaml)"
RGBD=/opt/orbslam3_ws/install/lib/ros2_orb_slam3/${SLAM_NODE:-rgbd_node_cpp}
POSE_BAG="${OUTDIR}/${LABEL}_poses"; mkdir -p "$OUTDIR"; rm -rf "$POSE_BAG"
echo "[run] LABEL=$LABEL CONFIG=$CNAME RATE=$RATE DOMAIN=$ROS_DOMAIN_ID"

# 1. ORB-SLAM3 RGB-D node, pointed straight at the raw TUM topics.
"$RGBD" --ros-args \
  -p voc_file_arg:="$VOC" \
  -p settings_file_path_arg:="$CDIR" \
  -p settings_name_arg:="$CNAME" \
  -p color_topic:=/camera/rgb/image_color \
  -p depth_topic:=/camera/depth/image ${EXTRA_PARAMS:-} > "${OUTDIR}/${LABEL}.slam.log" 2>&1 &
SPID=$!
for i in $(seq 1 120); do
  grep -qiE "node ready|ready to|Tracking started|Loading.*done" "${OUTDIR}/${LABEL}.slam.log" 2>/dev/null && break
  sleep 0.25
done
sleep 1

# 2. record estimate (sqlite3, not mcap)
ros2 bag record -s sqlite3 -o "$POSE_BAG" /slam/pose > "${OUTDIR}/${LABEL}.record.log" 2>&1 &
RPID=$!
sleep 2

# 3. play — backgrounded. ros2 bag play sometimes HANGS without exiting on this
# overlayfs host even after feeding every message, so we don't block on it; we
# drain on a stable recorded-pose count instead and then kill the player.
ros2 bag play "$BAG" -r "$RATE" > "${OUTDIR}/${LABEL}.play.log" 2>&1 &
PPID_BAG=$!

# 4. drain: wait until the recorded /slam/pose count stops growing.
DB3="${POSE_BAG}/${LABEL}_poses_0.db3"
last=-1; stable=0
poses() { python3 -c "import sqlite3;c=sqlite3.connect('file:$DB3?mode=ro',uri=True);print(c.execute('select count(*) from messages').fetchone()[0])" 2>/dev/null || echo 0; }
for i in $(seq 1 200); do
  kill -0 "$PPID_BAG" 2>/dev/null || { sleep 2; }   # player gone => feeding done
  n=$(poses); n=${n:-0}
  if [ "$n" -gt "$last" ]; then last=$n; stable=0; else stable=$((stable+1)); fi
  [ "$stable" -ge 4 ] && [ "$n" -gt 0 ] && break
  sleep 3
done
echo "[drain] poses=${last}"; sleep 2

# 5. shutdown — explicit PIDs. SIGINT recorder + wait (never kill -9 the recorder).
kill -9 "$PPID_BAG" 2>/dev/null || true
kill -INT "$RPID" 2>/dev/null || true
for i in $(seq 1 20); do kill -0 "$RPID" 2>/dev/null || break; sleep 1; done
kill -9 "$SPID" 2>/dev/null || true
wait 2>/dev/null || true

# 5. eval vs TUM groundtruth
python3 /splatograph/scripts/eval_tum.py "$POSE_BAG" --gt "$GT" \
  --label "$LABEL" --json "${OUTDIR}/${LABEL}.eval.json"
