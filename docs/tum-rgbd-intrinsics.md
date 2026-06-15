# TUM RGB-D intrinsics — freiburg1 / 2 / 3 are different cameras

The TUM RGB-D benchmark's `freiburg1`, `freiburg2`, `freiburg3` sequence families
were recorded in **separate campaigns with different Microsoft Kinect v1 units and
different calibrations**. They are **not** the same camera — using one family's
config on another warps the reprojection and inflates ATE. Always pick the config
by the sequence's `freiburgN` prefix.

| family | config | fx / fy | cx / cy | distortion (k1,k2,p1,p2,k3) |
|---|---|---|---|---|
| **fr1** | `config/TUM1_RGBD.yaml` | 517.31 / 516.47 | 318.64 / 255.31 | 0.2624, −0.9531, −0.0054, 0.0026, 1.1633 |
| **fr2** | *(add TUM2 if needed)* | 520.91 / 521.01 | 325.14 / 249.70 | 0.2312, −0.7849, −0.0033, −0.0001, 0.9172 |
| **fr3** | `config/TUM3_RGBD.yaml` | 535.40 / 539.20 | 320.10 / 247.60 | **0, 0, 0, 0, 0** |

Two reasons they differ:
1. **Different physical sensors / sessions** — each Kinect was checkerboard-
   calibrated separately, so fx and the principal point shift unit-to-unit.
2. **fr3 uses the "ROS default" pinhole with zero distortion.** TUM never shipped a
   custom radtan calibration for the freiburg3 sequences, so the benchmark
   convention (and ORB-SLAM3's stock `TUM3.yaml`) applies **no** distortion —
   whereas fr1/fr2 carry real, sizable coefficients (fr1's `k3` = 1.16 is large).

## Depth scale — the ros2-bag gotcha

The original TUM dataset ships **16-bit PNG depth in millimetres, factor 5000**
(`depth_metres = pixel / 5000`). **But** the ros2-converted bags on the share
publish `/camera/depth/image` as **`32FC1` already in METRES** (verified via
`ros2 topic echo /camera/depth/image --field encoding --once`). So in these
configs **`RGBD.DepthMapFactor: 1.0`**, not 5000. The gradslam/LC nodes take
`depth_scale:=1.0` for the same reason. Always re-check the encoding for a new
conversion before trusting geometry.

## Which sequence → which config (+ groundtruth)

| sequence | config | groundtruth.txt present? |
|---|---|---|
| `freiburg1_room` (loop) | TUM1_RGBD | yes |
| `freiburg3_long_office_household` (loop) | TUM3_RGBD | yes |
| `freiburg2_360_kidnap` (kidnap) | TUM2 (todo) | yes |
| `freiburg2_large_with_loop` (loop) | TUM2 (todo) | **no** → ATE not possible |

Run with `scripts/run_orbslam_tum.sh BAG CONFIG GT_TXT OUTDIR LABEL`; eval is
Umeyama-aligned ATE vs `groundtruth.txt` (`scripts/../eval_tum.py`).
