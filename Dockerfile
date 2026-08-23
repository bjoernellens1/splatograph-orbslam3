# Pinned splatograph-rgbd-sync GHCR image providing the authoritative
# splatograph_rgbd_msgs/RGBDSynced contract (issue #3 migration), pulled
# as a prebuilt image rather than git-cloned from source. splatograph-rgbd-sync
# is a PRIVATE repo -- an unauthenticated `git clone` of it fails in CI
# (confirmed: "fatal: could not read Username for 'https://github.com'"),
# and this repo's CI has no credential that would cover a cross-repo git
# clone anyway (GitHub-hosted runners, GITHUB_TOKEN scoped to this repo
# only). Its own CI (splatograph-rgbd-sync#17) now publishes this image on
# every push to main/tag, so pulling it needs only whatever pull access the
# package's own visibility/Actions-access grants -- see that repo's #16 for
# the remaining owner decision (public visibility vs. a per-repo grant).
# Bumped deliberately, not on every upstream push, so this build stays
# reproducible. Must be built from a commit including the [P0] "exact
# authoritative pair contract" rewrite (pair_ordinal, publish_stamp, the
# ns-precision fields) and the Jazzy `rclcpp::Time::to_msg()` fix
# (splatograph-rgbd-sync#14, in main as of 4dbf038 -- 32b8b4a, this
# Dockerfile's previous pin, predates that fix and does not compile on
# Jazzy at all, independent of the credential question above).
ARG RGBD_SYNC_IMAGE=ghcr.io/bjoernellens1/splatograph-rgbd-sync:sha-4dbf038

FROM ${RGBD_SYNC_IMAGE} AS rgbd_sync

FROM ros:jazzy-ros-base-noble AS builder

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV ROS_DISTRO=jazzy     DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends       build-essential cmake git ca-certificates pkg-config       libopencv-dev libeigen3-dev libboost-system-dev libboost-serialization-dev libssl-dev       libgl1-mesa-dev libglew-dev libxkbcommon-dev libwayland-dev wayland-protocols       libegl1-mesa-dev libc++-dev libepoxy-dev libjpeg-dev libpng-dev libtiff-dev       python3-colcon-common-extensions python3-natsort python3-wheel       ros-jazzy-ament-cmake ros-jazzy-ament-cmake-python ros-jazzy-cv-bridge       ros-jazzy-image-transport ros-jazzy-message-filters ros-jazzy-sensor-msgs ros-jazzy-std-msgs       ros-jazzy-rosidl-default-generators     && rm -rf /var/lib/apt/lists/*

WORKDIR /opt
RUN git clone --depth 1 --branch v0.9.5 https://github.com/stevenlovegrove/Pangolin.git Pangolin     && cmake -S Pangolin -B Pangolin/build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF     && cmake --build Pangolin/build -j"$(nproc)"     && cmake --install Pangolin/build

# splatograph_rgbd_msgs (the RGBDSynced contract) AND splatograph_rgbd_sync
# (the sync_node binary itself), copied prebuilt from the pinned
# splatograph-rgbd-sync image above instead of built from source here, so
# this image can run the full decode -> sync_node -> RGBDSynced -> ORB-SLAM3
# pipeline in one container (slam-launch's rgbd/rgbd_inertial modes below).
# Kept as a separate overlay workspace so it can also be COPY'd into the
# runtime image alongside orbslam3_ws.
COPY --from=rgbd_sync /opt/rgbd_sync_ws/install /opt/rgbd_sync_ws/install

WORKDIR /opt/orbslam3_ws/src
RUN git clone --depth 1 --branch jazzy https://github.com/Mechazo11/ros2_orb_slam3.git
# NOTE: previously this step did its own sed replacement of
# enablePangolinWindow/enableOpenCVWindow directly, and apply_patch.py below
# was NEVER ACTUALLY INVOKED in this Dockerfile (a pre-existing gap on main,
# found and fixed here as part of #3 -- the RGB-D/RGB-D-Inertial/stereo nodes
# apply_patch.py generates were never built into any shipped image). Now that
# apply_patch.py runs for real, its own common.cpp patch already disables
# both windows (see the "PATCHED: no X11"/"no GUI" edit in apply_patch.py) --
# the sed here was left in only long enough to discover, via a real build
# failure, that it pre-empties the exact snippet apply_patch.py matches on;
# removed rather than kept redundant.
COPY patches/apply_patch.py /tmp/apply_orbslam_patch.py
RUN python3 /tmp/apply_orbslam_patch.py /opt/orbslam3_ws/src/ros2_orb_slam3 /opt/orbslam3_ws/install
WORKDIR /opt/orbslam3_ws
# Overlay splatograph_rgbd_msgs so find_package(splatograph_rgbd_msgs) resolves.
RUN bash -lc 'source /opt/ros/${ROS_DISTRO}/setup.bash && source /opt/rgbd_sync_ws/install/setup.bash && colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release'

FROM ros:jazzy-ros-base-noble
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
LABEL org.opencontainers.image.source="https://github.com/bjoernellens1/splatograph-orbslam3"       org.opencontainers.image.description="ROS2 Jazzy ORB-SLAM3 container for Splatograph pose input"       org.opencontainers.image.licenses="GPL-3.0"
ENV ROS_DISTRO=jazzy     DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends       libopencv-dev libboost-system1.83.0 libboost-serialization1.83.0 libssl3       libgl1 libglew2.2 libepoxy0 libwayland-egl1 libwayland-cursor0 python3-natsort       ros-jazzy-cv-bridge ros-jazzy-image-transport ros-jazzy-message-filters ros-jazzy-sensor-msgs ros-jazzy-std-msgs     && rm -rf /var/lib/apt/lists/*
COPY --from=builder /usr/local /usr/local
# splatograph_rgbd_msgs runtime overlay (typesupport .so + Python bindings
# needed at runtime, not just build time) -- sourced by slam-launch.
COPY --from=builder /opt/rgbd_sync_ws/install /opt/rgbd_sync_ws/install
COPY --from=builder /opt/orbslam3_ws /opt/orbslam3_ws
COPY scripts/slam-launch /usr/local/bin/slam-launch
COPY scripts/rosbag-image-adapter.py /usr/local/bin/rosbag-image-adapter.py
COPY scripts/smoke.sh /usr/local/bin/splatograph-smoke
RUN echo /usr/local/lib > /etc/ld.so.conf.d/pangolin.conf     && ldconfig     && chmod +x /usr/local/bin/slam-launch /usr/local/bin/rosbag-image-adapter.py /usr/local/bin/splatograph-smoke
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["slam-launch"]
