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

## RGB-D tuning & setup

- **[`docs/orbbec-rgbd-setup.md`](docs/orbbec-rgbd-setup.md)** — how to run ORB-SLAM3
  *nicely* on Orbbec Femto RGB-D: the decompress pipeline, the tuned profile
  (`ThDepth 60`, `nFeatures 1250`, `DepthMapFactor 1000`), the depth-encoding and
  c2w-vs-w2c pose-convention traps, and per-bag results.
- [`docs/2026-06-13-orbbec-rgbd-tuning.md`](docs/2026-06-13-orbbec-rgbd-tuning.md)
  — the full parameter sweep behind those numbers.
- TUM RGB-D: `scripts/run_orbslam_tum.sh` + `config/TUM{1,3}_RGBD.yaml` (depth is
  32FC1 metres → `DepthMapFactor 1.0`); eval vs mocap GT via `scripts/../eval_tum.py`.
- **[`docs/tum-rgbd-intrinsics.md`](docs/tum-rgbd-intrinsics.md)** — why fr1/fr2/fr3
  are different cameras (per-family intrinsics + distortion), which config goes
  with which sequence, and the 32FC1-metres depth-scale gotcha.

## ROS Contract

Default input/output topics are documented in `config/default.yaml`. Provider output is normalized for Splatograph around `/slam/pose`, `/slam/odom`, `/slam/path`, and `/tf` where the upstream method publishes those streams.

## Upstream

- Upstream: https://github.com/Mechazo11/ros2_orb_slam3
- Pinned reference for initial implementation: `jazzy@0ab45b1cdc93d0be49544841a451b083c482b92a`
- ROS distro: Jazzy
- Platform: `linux/amd64`
- Runtime policy: CPU/AMD-compatible, no NVIDIA runtime dependency

## Ecosystem

This is part of the **Splatograph** streaming 3DGS stack. See
[bjoernellens1/splatograph/docs/ECOSYSTEM_CONTRIBUTIONS.md](https://github.com/bjoernellens1/splatograph/blob/main/docs/ECOSYSTEM_CONTRIBUTIONS.md)
for the full dependency graph, per-repo contributions, and AMD/ROCm
(gfx1151) + NVIDIA/CUDA port notes.

## References

- **ORB-SLAM3** — C. Campos, R. Elvira, J. J. G. Rodríguez, J. M. M. Montiel,
  J. D. Tardós, *"ORB-SLAM3: An Accurate Open-Source Library for Visual,
  Visual–Inertial, and Multimap SLAM,"* IEEE Transactions on Robotics, 37(6),
  2021. [arXiv:2007.11898](https://arxiv.org/abs/2007.11898).
  Original code: https://github.com/UZ-SLAMLab/ORB_SLAM3
- **ORB-SLAM (foundational)** — R. Mur-Artal, J. M. M. Montiel, J. D. Tardós,
  *"ORB-SLAM: A Versatile and Accurate Monocular SLAM System,"* IEEE T-RO, 31(5),
  2015. [arXiv:1502.00956](https://arxiv.org/abs/1502.00956).
- **ROS2 port (upstream used here)** — A. K. Sahu et al., `ros2_orb_slam3`:
  https://github.com/Mechazo11/ros2_orb_slam3

## Smoke Test

```bash
docker build -t ghcr.io/bjoernellens1/splatograph-orbslam3:jazzy .
docker run --rm ghcr.io/bjoernellens1/splatograph-orbslam3:jazzy splatograph-smoke
docker compose config
```
