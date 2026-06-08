# Splatograph ORB-SLAM3 ROS2 Jazzy Container

CPU/AMD-compatible ROS2 Jazzy ORB-SLAM3 image for Splatograph pose input.

## Image

```bash
docker pull ghcr.io/bjoernellens1/splatograph-orbslam3:jazzy
```

## Run

```bash
docker compose up slam
```

For bag replay from `./bags`:

```bash
BAG_PATH=/bags/input docker compose --profile bag up bag slam
```

For Splatograph integration:

```bash
docker compose -f compose.splatograph.yml up
```

## ROS Contract

Default input/output topics are documented in `config/default.yaml`. Provider output is normalized for Splatograph around `/slam/pose`, `/slam/odom`, `/slam/path`, and `/tf` where the upstream method publishes those streams.

## Upstream

- Upstream: https://github.com/Mechazo11/ros2_orb_slam3
- Pinned reference for initial implementation: `jazzy@0ab45b1cdc93d0be49544841a451b083c482b92a`
- ROS distro: Jazzy
- Platform: `linux/amd64`
- Runtime policy: CPU/AMD-compatible, no NVIDIA runtime dependency

## Smoke Test

```bash
docker build -t ghcr.io/bjoernellens1/splatograph-orbslam3:jazzy .
docker run --rm ghcr.io/bjoernellens1/splatograph-orbslam3:jazzy splatograph-smoke
docker compose config
```
