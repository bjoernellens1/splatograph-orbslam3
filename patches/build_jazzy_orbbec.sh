#!/usr/bin/env bash
# Build ghcr.io/bjoernellens1/splatograph-orbslam3:jazzy-orbbec
# by patching the existing :jazzy image in place:
#   1. install boost dev headers (DBoW2 / g2o headers live in -dev, runtime image only has .so)
#   2. fresh-clone the upstream jazzy branch
#   3. apply 4 edits: common.hpp + common.cpp + package.xml + CMakeLists.txt
#   4. colcon build --merge-install
#   5. remove -dev packages to keep the final image small
#   6. commit as :jazzy-orbbec
#
# Usage: bash patches/build_jazzy_orbbec.sh [BASE_TAG] [NEW_TAG]
#   defaults: BASE_TAG=jazzy, NEW_TAG=jazzy-orbbec
set -euo pipefail

BASE_TAG="${1:-jazzy}"
NEW_TAG="${2:-jazzy-orbbec-v3}"
IMAGE="ghcr.io/bjoernellens1/splatograph-orbslam3:${BASE_TAG}"
NEW_IMAGE="ghcr.io/bjoernellens1/splatograph-orbslam3:${NEW_TAG}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

if podman image exists "${NEW_IMAGE}"; then
  echo "${NEW_IMAGE} already exists locally; remove it first: podman rmi ${NEW_IMAGE}" >&2
  exit 1
fi

echo "[1/6] starting long-lived container from ${IMAGE}"
TMP_CID=$(podman run -d --entrypoint sleep "${IMAGE}" infinity)
trap 'podman rm -f "${TMP_CID}" >/dev/null 2>&1 || true' EXIT

echo "[2/6] install dev headers (boost, ssl, epoxy) + message_filters"
podman exec "${TMP_CID}" bash -c "set -e
  export DEBIAN_FRONTEND=noninteractive
  apt-get update
  apt-get install -y --no-install-recommends \
    libboost-system-dev libboost-serialization-dev libssl-dev \
    libepoxy-dev libgl1-mesa-dev libglew-dev \
    ros-jazzy-message-filters
  rm -rf /var/lib/apt/lists/*
"

echo "[3/6] fresh-clone upstream jazzy branch"
podman exec "${TMP_CID}" bash -c "set -e
  rm -rf /opt/orbslam3_ws/src/ros2_orb_slam3
  cd /opt/orbslam3_ws/src
  git clone --depth 1 --branch jazzy https://github.com/Mechazo11/ros2_orb_slam3.git
"

echo "[4/6] apply patches"
podman cp "${SCRIPT_DIR}/apply_patch.py" "${TMP_CID}:/tmp/apply_patch.py"
podman exec "${TMP_CID}" python3 /tmp/apply_patch.py /opt/orbslam3_ws/src/ros2_orb_slam3 /opt/orbslam3_ws/install

echo "[5/6] colcon build --merge-install (long step)"
podman exec "${TMP_CID}" bash -c "set -e
  export DEBIAN_FRONTEND=noninteractive
  apt-get install -y --no-install-recommends \
    libboost-system-dev libboost-serialization-dev libssl-dev \
    libepoxy-dev libgl1-mesa-dev libglew-dev \
    ros-jazzy-message-filters
  rm -rf /var/lib/apt/lists/*
  cd /opt/orbslam3_ws
  bash -lc 'source /opt/ros/\${ROS_DISTRO}/setup.bash && colcon build --merge-install --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release --install-base /opt/orbslam3_ws/install --packages-select ros2_orb_slam3 2>&1'
"

echo "[6/6] install runtime deps (cv-bridge + ros2cli) + clean dev + commit ${NEW_IMAGE}"
podman exec "${TMP_CID}" bash -c "set -e
  export DEBIAN_FRONTEND=noninteractive
  # Re-install runtime deps that the base image is missing (cv-bridge is
  # required by the orbslam3 C++ node; ros2cli is required by the slam
  # launch wrapper to find the executables).
  apt-get install -y --no-install-recommends \
    ros-jazzy-cv-bridge \
    ros-jazzy-ros2cli ros-jazzy-ros2pkg ros-jazzy-ros2node ros-jazzy-ros2topic \
    ros-jazzy-ros2service ros-jazzy-ros2action ros-jazzy-ros2param \
    ros-jazzy-ros2lifecycle ros-jazzy-ros2multicast ros-jazzy-ros2bag \
    ros-jazzy-ros2component ros-jazzy-ros2interface ros-jazzy-ros2launch \
    ros-jazzy-ros2doctor
  # NOTE: We intentionally keep the build-time dev headers in the image.
  # Purging them triggers an apt cascade that removes ros-jazzy-rclcpp and
  # ros-jazzy-cv-bridge even after apt-mark manual, because the base image
  # has APT::Get::AutomaticRemove enabled.  The dev headers are ~5 MB — not
  # worth the breakage.
  rm -rf /var/lib/apt/lists/*
  rm -rf /opt/orbslam3_ws/src/ros2_orb_slam3/.git
  rm -f /tmp/apply_patch.py
"
# Install slam-launch and default camera configs
podman cp "${SCRIPT_DIR}/../scripts/slam-launch" "${TMP_CID}:/usr/local/bin/slam-launch"
podman exec "${TMP_CID}" chmod +x /usr/local/bin/slam-launch
podman exec "${TMP_CID}" mkdir -p /config
podman cp "${SCRIPT_DIR}/../config/OrbbecFemtoMega.yaml" "${TMP_CID}:/config/OrbbecFemtoMega.yaml"
podman cp "${SCRIPT_DIR}/../config/OrbbecFemtoMega_RGBD.yaml" "${TMP_CID}:/config/OrbbecFemtoMega_RGBD.yaml"
# Make sure the COLCON_PREFIX_PATH includes orbslam3_ws/install
podman exec "${TMP_CID}" bash -c "grep -q orbslam3_ws /opt/ros/jazzy/setup.bash || echo 'source /opt/orbslam3_ws/install/setup.bash' >> /opt/ros/jazzy/setup.bash"
podman stop "${TMP_CID}" >/dev/null
podman commit --change ENTRYPOINT='["/ros_entrypoint.sh"]' --change CMD='["slam-launch"]' "${TMP_CID}" "${NEW_IMAGE}" >/dev/null
podman rm "${TMP_CID}" >/dev/null
trap - EXIT

echo "done. new image:"
podman images --format "table {{.Repository}}:{{.Tag}}\t{{.Size}}\t{{.CreatedSince}}" | grep -E "(TAG|${NEW_TAG})" || true
