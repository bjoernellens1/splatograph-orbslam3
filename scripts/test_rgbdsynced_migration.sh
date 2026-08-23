#!/usr/bin/env bash
# Host-side (no Docker/colcon required) regression test for the #3
# RGBDSynced migration: checks the *source of truth* patches/apply_patch.py
# and scripts/decompress_rgbd_node.py directly, so it can run fast in CI
# before (or without) a full container build.
#
# Complements scripts/smoke.sh, which checks the same properties against the
# actually-built container (binary presence, generated source on disk).
set -eo pipefail
cd "$(dirname "$0")/.."

fail=0
check() {
    local desc="$1" cmd="$2"
    if eval "$cmd"; then
        echo "PASS: $desc"
    else
        echo "FAIL: $desc" >&2
        fail=1
    fi
}

# rgbd_node_cpp / rgbd_inertial_node_cpp must consume RGBDSynced, not
# independently re-pair color/depth.
check "apply_patch.py declares rgbd_synced_topic param" \
    'grep -q "rgbd_synced_topic" patches/apply_patch.py'
check "apply_patch.py includes splatograph_rgbd_msgs/msg/rgbd_synced.hpp" \
    'grep -q "splatograph_rgbd_msgs/msg/rgbd_synced.hpp" patches/apply_patch.py'
check "RGBD_NODE_CPP / RGBD_INERTIAL_NODE_CPP do not declare color_topic/depth_topic" \
    '! grep -qE "declare_parameter\(\"(color|depth)_topic\"" patches/apply_patch.py'
check "package.xml patch adds splatograph_rgbd_msgs dependency" \
    'grep -q "splatograph_rgbd_msgs</build_depend>" patches/apply_patch.py'
check "CMakeLists.txt patch adds find_package(splatograph_rgbd_msgs)" \
    'grep -q "find_package(splatograph_rgbd_msgs REQUIRED)" patches/apply_patch.py'

# ApproximateTime must be gone as an actual sync mechanism from the RGB-D-
# consuming nodes (stereo nodes are out of scope for RGBDSynced but were
# upgraded to ExactTime -- checked below). Checks real code usage
# (message_filters::sync_policies::ApproximateTime<...>), not just the bare
# word -- this repo's own migration comments legitimately mention
# "ApproximateTime" in prose explaining what was removed and why.
check "no message_filters::sync_policies::ApproximateTime usage anywhere" \
    '! grep -q "message_filters::sync_policies::ApproximateTime<" patches/apply_patch.py'
check "no #include .../approximate_time.h anywhere" \
    '! grep -q "sync_policies/approximate_time.h" patches/apply_patch.py'
check "stereo nodes use ExactTime" \
    'grep -q "message_filters::sync_policies::ExactTime<" patches/apply_patch.py'

# decompress_rgbd_node.py must be decode-only (no message_filters-based
# re-pairing mechanism; the word "ApproximateTime" is allowed to remain in
# the file's own migration-history docstring).
check "decompress_rgbd_node.py has no ApproximateTimeSynchronizer usage" \
    '! grep -q "ApproximateTimeSynchronizer(" scripts/decompress_rgbd_node.py'
check "decompress_rgbd_node.py has no sync parameter" \
    '! grep -q "declare_parameter(\"sync\"" scripts/decompress_rgbd_node.py'

# Dockerfile must vendor splatograph_rgbd_msgs + splatograph_rgbd_sync into
# the ORB-SLAM3 overlay. splatograph-rgbd-sync is a private repo, so this is
# pulled as a prebuilt image (splatograph-rgbd-sync#16/#17) rather than
# git-cloned+built from source -- an unauthenticated git clone of a private
# repo fails in CI, which is exactly what this migration originally hit.
check "Dockerfile pins RGBD_SYNC_IMAGE" \
    'grep -q "ARG RGBD_SYNC_IMAGE=" Dockerfile'
check "Dockerfile pulls splatograph-rgbd-sync as a prebuilt GHCR image" \
    'grep -q "ghcr.io/bjoernellens1/splatograph-rgbd-sync" Dockerfile'
check "Dockerfile copies rgbd_sync_ws install from that image" \
    'grep -q "COPY --from=rgbd_sync /opt/rgbd_sync_ws/install" Dockerfile'
check "Dockerfile now invokes apply_patch.py (was previously never called on main)" \
    'grep -q "apply_orbslam_patch.py" Dockerfile'

# slam-launch must launch sync_node and pass rgbd_synced_topic, not color/depth.
check "slam-launch passes rgbd_synced_topic to rgbd_node_cpp" \
    'grep -q "\-p rgbd_synced_topic:=\${RGBD_SYNCED_TOPIC}" scripts/slam-launch'
check "slam-launch launches sync_node" \
    'grep -q "SYNC_NODE=/opt/rgbd_sync_ws/install/lib/splatograph_rgbd_sync/sync_node" scripts/slam-launch'

if [ "$fail" -ne 0 ]; then
    echo "test_rgbdsynced_migration.sh: FAILURES ABOVE" >&2
    exit 1
fi
echo "test_rgbdsynced_migration.sh: all checks passed"
