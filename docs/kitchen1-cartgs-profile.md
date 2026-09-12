# kitchen1 CartGS Profile Comparison

## Configurations Tested

| Config | ThDepth | nFeatures | Stereo.b | ATE (m) | RPE Trans (m) | Coverage | Status |
|--------|---------|-----------|----------|---------|---------------|----------|--------|
| tuned  | 60.0    | 1250      | 0.05     | 0.0451  | 0.0111        | 2055/2055| ✓ BEST |
| cartgs | 40.0    | 1200      | 0.05562  | 0.0492  | 0.0112        | 2055/2055| ✓      |

## Best Configuration (Tuned)
- **Basis**: Recommended Orbbec profile from orbbec-rgbd-setup.md
- **ThDepth Rationale**: 60.0 m boundary (3m x 20 depth/depth_max) excludes noisy far returns
- **Results**: ATE 45.1 mm, RPE 11.1 mm, 100% tracking coverage
- **Staged**: kitchen1_orbposes_v2

## Method
- **Bag**: kitchen1_o3dpose_injected (2055 frames @ 30fps)
- **Reference**: O3D /camera_pose (odometry, non-ground-truth)
- **Scoring**: ATE/RPE via Umeyama SE(3) alignment
- **Runtime**: CPU only, rmw_fastrtps_cpp middleware
