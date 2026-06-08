FROM ros:jazzy-ros-base-noble AS builder

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
ENV ROS_DISTRO=jazzy     DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y --no-install-recommends       build-essential cmake git ca-certificates pkg-config       libopencv-dev libeigen3-dev libboost-system-dev libboost-serialization-dev libssl-dev       libgl1-mesa-dev libglew-dev libxkbcommon-dev libwayland-dev wayland-protocols       libegl1-mesa-dev libc++-dev libepoxy-dev libjpeg-dev libpng-dev libtiff-dev       python3-colcon-common-extensions python3-natsort python3-wheel       ros-jazzy-ament-cmake ros-jazzy-ament-cmake-python ros-jazzy-cv-bridge       ros-jazzy-image-transport ros-jazzy-sensor-msgs ros-jazzy-std-msgs     && rm -rf /var/lib/apt/lists/*

WORKDIR /opt
RUN git clone --depth 1 --branch v0.9.5 https://github.com/stevenlovegrove/Pangolin.git Pangolin     && cmake -S Pangolin -B Pangolin/build -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF -DBUILD_EXAMPLES=OFF     && cmake --build Pangolin/build -j"$(nproc)"     && cmake --install Pangolin/build

WORKDIR /opt/orbslam3_ws/src
RUN git clone --depth 1 --branch jazzy https://github.com/Mechazo11/ros2_orb_slam3.git
WORKDIR /opt/orbslam3_ws
RUN bash -lc 'source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release'

FROM ros:jazzy-ros-base-noble
SHELL ["/bin/bash", "-o", "pipefail", "-c"]
LABEL org.opencontainers.image.source="https://github.com/bjoernellens1/splatograph-orbslam3"       org.opencontainers.image.description="ROS2 Jazzy ORB-SLAM3 container for Splatograph pose input"       org.opencontainers.image.licenses="GPL-3.0"
ENV ROS_DISTRO=jazzy     DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends       libopencv-dev libboost-system1.83.0 libboost-serialization1.83.0 libssl3       libgl1 libglew2.2 libepoxy0 python3-natsort       ros-jazzy-cv-bridge ros-jazzy-image-transport ros-jazzy-sensor-msgs ros-jazzy-std-msgs     && rm -rf /var/lib/apt/lists/*
COPY --from=builder /usr/local /usr/local
COPY --from=builder /opt/orbslam3_ws /opt/orbslam3_ws
COPY scripts/slam-launch /usr/local/bin/slam-launch
COPY scripts/smoke.sh /usr/local/bin/splatograph-smoke
RUN chmod +x /usr/local/bin/slam-launch /usr/local/bin/splatograph-smoke
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["slam-launch"]
