#!/usr/bin/env bash
# Pose-injection preset: run tuned ORB-SLAM3 RGB-D on a POSE-LESS bag and write a
# NEW bag = original data verbatim + /camera_pose + rich provenance.
#
# Usage: inject_poses.sh RAW_BAG CONFIG_YAML OUT_BAG LABEL [RATE] [HARDWARE] [DURATION_S]
# Env: GIT_SHA, ORBSLAM_IMAGE (-> provenance), ROS_DOMAIN_ID.
set -eo pipefail
RAW="$1"; CONFIG="$2"; OUT="$3"; LABEL="${4:-inject}"; RATE="${5:-1.0}"
HW="${6:-Orbbec Femto (Bolt/Mega)}"; DURATION="${7:-0}"
WORK="$(cd "$(dirname "$OUT")" && pwd)"
PROV="${WORK}/${LABEL}.provenance.json"

# 1. run ORB-SLAM3 over the raw bag; records /slam/pose (no /camera_pose to compare)
bash /scripts/run_orbslam_rgbd.sh "$RAW" "$CONFIG" "$WORK" "${LABEL}_run" "$RATE" "$DURATION"
POSES="${WORK}/${LABEL}_run_poses"

# 2. build rich provenance (hardware + full config + tooling)
python3 /scripts/make_provenance.py --bag "$RAW" --config "$CONFIG" \
  --hardware "$HW" --slam-mode rgbd --out "$PROV"

# 3. merge: original verbatim + /camera_pose + latched /splatograph/provenance
rm -rf "$OUT"
python3 /scripts/inject_poses.py --orig "$RAW" --poses "$POSES" --out "$OUT" --provenance "$PROV"
cp "$PROV" "${OUT}.provenance.json"   # sidecar
echo "[inject] done: $OUT (+ sidecar ${OUT}.provenance.json)"
