# kitchen1 CartGS Profile Comparison

## Executive Summary

ORB-SLAM3 FINAL (loop-closed, post-BA) trajectories achieve **3.7× better ATE RMSE** than causal online streams, demonstrating the value of pose-graph optimization. **Tuned config** selected for official baseline.

## Causal (Online) Results

| Config | ThDepth | nFeatures | ATE RMSE (mm) | ATE Mean (mm) | RPE Trans (mm) | Frames | Notes |
|--------|---------|-----------|---------------|---------------|----------------|--------|-------|
| tuned  | 60.0    | 1250      | 45.1          | 36.9          | 11.1           | 2055   | ✓ BEST |
| cartgs | 40.0    | 1200      | 49.2          | 40.0          | 11.2           | 2055   | ✓      |

Causal (live /slam/pose during playback) evaluated vs O3D /camera_pose reference via inject_poses.py + eval_traj.py.

## FINAL (Loop-Closed) Results

| Config | ThDepth | nFeatures | ATE RMSE (mm) | ATE Mean (mm) | RPE Trans (mm) | Frames | Improvement |
|--------|---------|-----------|---------------|---------------|----------------|--------|-------------|
| tuned  | 60.0    | 1250      | 12.31         | 10.25         | 9.44           | 2050   | 3.7×        |
| cartgs | 40.0    | 1200      | 17.69         | 13.52         | 9.72           | 2055   | 2.8×        |

FINAL trajectories (SaveTrajectoryTUM at SIGINT shutdown) interpolated to reference timestamps, SE(3)-aligned via Umeyama (scale=1), scored vs OpenCV reference convention.

## Selected Configuration (Tuned)

- **Basis**: Recommended Orbbec profile from orbbec-rgbd-setup.md
- **ThDepth**: 60.0 (3 m boundary; excludes noisy far returns >3 m)
- **nFeatures**: 1250 (saturates tracking; further increase adds no benefit)
- **FINAL ATE RMSE**: 12.31 mm (vs OpenCV reference)
- **Keyframes**: 217 out of 2050 tracked frames
- **Staged**: `/mnt/cps_scratch1_tmp/bjoern/output/icra27-custom-tum-v1/kitchen1_orbposes_v2`

## Method

- **Bag**: kitchen1_o3dpose_injected (1280×720 @30 fps, 2055 frames, 68 s)
- **Image**: splatograph-orbslam3:k1final (CPU-only, built with SaveTrajectoryTUM patch)
- **Pipeline**: decompress_rgbd_node.py → rgbd_node_cpp (graceful SIGINT) → eval_traj.py
- **Middleware**: rmw_fastrtps_cpp, ROS_DOMAIN_ID=201
- **Reference**: OpenCV Z-forward convention; TUM format (timestamp tx ty tz qx qy qz qw)
