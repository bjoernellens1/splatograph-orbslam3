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
# RATE default 1.0: profiling showed ORB-SLAM3 RGB-D is real-time at 30fps
# (29.9 pose/s, 100% coverage at -r 1.0), so -r 0.5 only wastes time.
BAG="$1"; CONFIG="$2"; OUTDIR="$3"; LABEL="${4:-run}"; RATE="${5:-1.0}"; DURATION="${6:-0}"
PLAY_EXTRA=""
[ "$DURATION" != "0" ] && PLAY_EXTRA="--playback-duration $DURATION"
# Set EXTERNAL_RGBD_SYNC=1 when one already-running decode/rectify/SyncNode
# pipeline owns the authoritative RGBDSynced stream for multiple consumers.
# This is the only supported way to feed Splatograph and ORB-SLAM3 from the
# same source identities; do not start a second local SyncNode in that case.
EXTERNAL_RGBD_SYNC="${EXTERNAL_RGBD_SYNC:-0}"
RGBD_SYNCED_TOPIC="${RGBD_SYNCED_TOPIC:-/camera/rgbd_synced}"

source /opt/ros/jazzy/setup.bash
source /opt/rgbd_sync_ws/install/setup.bash
source /opt/orbslam3_ws/install/setup.bash
export PYTHONPATH="/splatograph:${PYTHONPATH:-}"
VOC=/opt/orbslam3_ws/src/ros2_orb_slam3/orb_slam3/Vocabulary/ORBvoc.txt.bin
CDIR="$(cd "$(dirname "$CONFIG")" && pwd)/"
CNAME="$(basename "$CONFIG" .yaml)"
# SLAM_NODE: rgbd_node_cpp (default) or rgbd_inertial_node_cpp (Orbbec VIO).
# EXTRA_PARAMS: e.g. "-p imu_topic:=/camera/imu" for the inertial node --
# NOTE: since the #3 RGBDSynced migration, rgbd_inertial_node_cpp has no
# imu_topic parameter any more (IMU arrives inside RGBDSynced.imu_samples);
# EXTRA_PARAMS is kept for forward compat but stale imu_topic overrides here
# are now a no-op, not an error (ROS2 silently ignores unused -p args).
RGBD=/opt/orbslam3_ws/install/lib/ros2_orb_slam3/${SLAM_NODE:-rgbd_node_cpp}
SYNC_NODE=/opt/rgbd_sync_ws/install/lib/splatograph_rgbd_sync/sync_node
POSE_BAG="${OUTDIR}/${LABEL}_poses"
mkdir -p "$OUTDIR"; rm -rf "$POSE_BAG"

echo "[run] LABEL=$LABEL CONFIG=$CONFIG RATE=$RATE DOMAIN=$ROS_DOMAIN_ID"
now() { date +%s.%N; }
T_start=$(now)
INIT_TIMEOUT="${INIT_WAIT:-30}"

DPID=""; YPID=""
if [ "$EXTERNAL_RGBD_SYNC" != "1" ]; then
  # 1. decode-only transport stage; SyncNode owns all colour/depth pairing.
  python3 /scripts/decompress_rgbd_node.py --ros-args \
    -p color_in:=/camera/color/image_raw/compressed \
    -p depth_in:=/camera/depth/image_raw/compressed \
    -p color_out:=/camera/color/image_raw \
    -p depth_out:=/camera/depth/image_raw \
    -p color_encoding:=bgr8 > "${OUTDIR}/${LABEL}.decompress.log" 2>&1 &
  DPID=$!
  "$SYNC_NODE" --ros-args \
    -p color_topic:=/camera/color/image_raw \
    -p depth_topic:=/camera/depth/image_raw \
    -p imu_topic:=/camera/imu \
    -r rgbd_synced:="$RGBD_SYNCED_TOPIC" > "${OUTDIR}/${LABEL}.sync.log" 2>&1 &
  YPID=$!
fi

# 2. ORB-SLAM3 RGB-D node (loads vocab). Wait for the node's "ready" log line
#    (measures real init/vocab-load time) instead of a fixed conservative sleep.
"$RGBD" --ros-args \
  -p voc_file_arg:="$VOC" \
  -p settings_file_path_arg:="$CDIR" \
  -p settings_name_arg:="$CNAME" \
  -p rgbd_synced_topic:="$RGBD_SYNCED_TOPIC" ${EXTRA_PARAMS:-} > "${OUTDIR}/${LABEL}.slam.log" 2>&1 &
SPID=$!
SLAM_LOG="${OUTDIR}/${LABEL}.slam.log"
for i in $(seq 1 "$((INIT_TIMEOUT*4))"); do
  grep -qiE "node ready|ready to|Tracking started|Loading.*done" "$SLAM_LOG" 2>/dev/null && break
  sleep 0.25
done
T_ready=$(now)

# 3. record estimate + reference
ros2 bag record -s mcap -o "$POSE_BAG" /slam/pose /camera_pose > "${OUTDIR}/${LABEL}.record.log" 2>&1 &
RPID=$!
sleep 2

# 4. play the bag (blocks until end)
T_play0=$(now)
ros2 bag play "$BAG" -r "$RATE" $PLAY_EXTRA > "${OUTDIR}/${LABEL}.play.log" 2>&1
T_play1=$(now)
sleep 4

# 5. shut down recorder + slam + decompress
kill -INT "$RPID" 2>/dev/null || true; sleep 3
PIPELINE_PIDS=("$SPID")
[ -n "$YPID" ] && PIPELINE_PIDS+=("$YPID")
[ -n "$DPID" ] && PIPELINE_PIDS+=("$DPID")
kill -INT "${PIPELINE_PIDS[@]}" 2>/dev/null || true; sleep 2
kill "${PIPELINE_PIDS[@]}" "$RPID" 2>/dev/null || true
wait 2>/dev/null || true

# 6. evaluate trajectory vs O3D reference
T_eval0=$(now)
python3 /scripts/eval_traj.py "$POSE_BAG" --label "$LABEL" --json "${OUTDIR}/${LABEL}.eval.json"
T_eval1=$(now)

# 7. timing breakdown — find real blockers
DECOMP=$(grep -aoE "color=[0-9]+ depth=[0-9]+" "${OUTDIR}/${LABEL}.decompress.log" | tail -1)
VOCLINE=$(grep -aiE "vocabulary loaded|loading orb voc" "$SLAM_LOG" | tail -1)
N_EST=$(grep -aoE '"n_est": [0-9]+' "${OUTDIR}/${LABEL}.eval.json" 2>/dev/null | grep -oE "[0-9]+" | head -1)
awk -v a="$T_start" -v b="$T_ready" -v c="$T_play0" -v d="$T_play1" -v e="$T_eval0" -v f="$T_eval1" \
    -v n="${N_EST:-0}" -v lbl="$LABEL" 'BEGIN{
  init=b-a; play=d-c; ev=f-e; fps=(play>0&&n>0)?n/play:0;
  printf "[timing] %s init=%.1fs play=%.1fs(%d poses, %.1f pose/s) eval=%.1fs\n", lbl, init, play, n, fps, ev
}'
[ -n "$DECOMP" ] && echo "[timing] $LABEL decompress: $DECOMP"
[ -n "$VOCLINE" ] && echo "[timing] $LABEL voc: $VOCLINE"
