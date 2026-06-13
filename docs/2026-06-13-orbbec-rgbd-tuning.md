# Orbbec Femto RGB-D ORB-SLAM3 trajectory tuning (2026-06-13)

Tune ORB-SLAM3 RGB-D to **agree** with the baked-in `/camera_pose` reference
(NVIDIA-accelerated Open3D RGBD frame-to-model odometry from another PC). The
reference is **not ground truth** — it is O3D odometry; the metric is *agreement*
(ATE-vs-O3D), which will not reach zero and penalises ORB-SLAM3 where it diverges
from O3D's own drift. `kitchen1_slam` + `workshop1_slam` are **Femto Bolt**;
Bolt and Mega share optics, so one profile serves both (intrinsics per-unit).

## Harness (modular, ROS2; `scripts/`)
`ros2 bag play` → `decompress_rgbd_node.py` (JPEG colour + PNG-16UC1 depth →
raw Image) → `rgbd_node_cpp` (per-bag config from `make_orbbec_config.py`) →
record `/slam/pose` vs `/camera_pose` → `eval_traj.py` (ATE/RPE via
`splatograph/utils/trajectory_eval.py` Umeyama SE(3) + coverage + track-losses).
`run_orbslam_rgbd.sh` runs one pass; `sweep.sh` tabulates a coarse one-factor
sweep (median-of-N), ranked coverage-first then ATE.

## Data checks (Orbbec publisher doesn't declare these)
- **Depth↔colour ALIGNED**: same 1280×720, same `frame_id`
  (`camera_color_optical_frame`), same K → feed depth directly, no reprojection.
- **Undistortion**: camera_info shows `P==K, R==I, D!=0` ⇒ raw (distorted) images
  ⇒ apply distortion. Empirically (D-on vs D-off): 53.7 vs 54.3 mm ATE — within
  the ~2 mm run noise, so distortion barely affects *trajectory* (matters more
  for rendering). Distortion is applied (correct + not worse).
- **Depth histogram** (workshop1): p50=2.98 m, p99=4.03 m; **87 % of depth beyond
  the old 2.0 m close/far boundary** (ThDepth=40) → ThDepth was the lead axis.

## Sweep (workshop1_slam, 60 s, 2 reps; ATE-vs-O3D)
| config | ATE med (mm) | coverage | losses |
|---|---|---|---|
| **ThDepth=60, nFeat=1250** | **51.1** | 100 % | 0 |
| ThDepth=40, nFeat=1250 (baseline) | 54.0 | 99.9 % | 0 |
| ThDepth=80, nFeat=1250 | 54.8 | 99.8 % | 0–1 |
| ThDepth=100, nFeat=2000 | 54.8 | 99.9 % | 0–1 |
| ThDepth=80, nFeat=2000 | 58.4 | 100 % | 0 |

**Tuned profile = ThDepth 60 (3.0 m), nFeatures 1250** (scaleFactor 1.2, nLevels
8, FAST 20/7, b 0.05, distortion-on). Going past 60 m adds noisy far Femto depth
(3–4 m) that *hurts*; more features don't help. ⇒ `config/OrbbecFemtoBolt_RGBD.yaml`
(= `OrbbecFemtoMega_RGBD.yaml`). Run-to-run variance ~2 mm (3× repeat).

## Validation
- workshop1 60 s: 50–52 mm, 100 % coverage. kitchen1 **full bag: 43 mm**, 100 %,
  0 losses → profile generalises across scenes.
- workshop1 **full bag: ATE 235 mm but RPE only 8 mm** (100 % coverage, 0 losses).
  High global ATE + low local RPE = the two trajectories agree locally and drift
  apart globally over the full run — the expected O3D-reference limitation (O3D is
  drifting odometry without loop closure; ORB-SLAM3 with BA/loop-closure can be
  *more* correct yet disagree more). Not a tuning regression.

## Blocker analysis (the real fixes)
Per-stage `[timing]`:
- **Init (vocab load) = 0.3 s** — the `.bin` vocabulary loads instantly. The
  harness previously slept a fixed **15 s** per run (pure waste). Fixed:
  wait-for-`node ready` log line → 0.3 s.
- **`-r 1.0` is real-time**: 29.9 pose/s = full 30 fps, 100 % coverage, 0 losses,
  ATE ≈ the `-r 0.5` result. SLAM tracking and decompress both keep up at 30 fps.
  ⇒ the old `-r 0.5` doubled wall time for nothing. Default rate is now **1.0**.
- Eval 0.2 s, decompress 30 fps — negligible.
- **Net: runs are ~2× faster** (real-time replay + 15 s→0.3 s init).

## Multi-mode support (image `:jazzy-multimode`)
Added three ORB-SLAM3 ROS2 nodes (via `patches/apply_patch.py`, same mechanism
as the RGB-D node): `rgbd_inertial_node_cpp` (Orbbec VIO, `IMU_RGBD`),
`stereo_node_cpp` (`STEREO`), `stereo_inertial_node_cpp` (`IMU_STEREO`). All five
nodes compile/install; `slam-launch` gains `rgbd_inertial`, `stereo`,
`stereo_inertial` modes.

- **Orbbec RGBD-Inertial (VIO)** — node runs and tracks **real-time** (30 pose/s,
  99.8 % coverage) on workshop1_slam, so the mode works. But with **nominal IMU
  noise + tf-derived Tbc it is untuned**: ATE-vs-O3D **1.10 m** and RPE 98 mm vs
  RGB-D-only's 51 mm / 5 mm — the IMU currently *hurts*. On these well-textured
  RGB-D scenes RGB-D-only is already excellent; VIO needs IMU calibration (Allan
  variance for the noise params, verify `IMU.T_b_c1`, tune VI-init excitation)
  before it helps. Answer to "can we do Orbbec VIO": **yes (RGBD-Inertial), and
  it runs — but it must be IMU-calibrated to beat RGB-D-only.**
- **RealSense D435i stereo / stereo-inertial** — nodes launch and parse the
  template configs (stereo loads both cameras; stereo-inertial also loads the IMU
  calibration). Full tracking validation deferred: no D435i IR-stereo + IMU + GT
  dataset (QueensCAMP is D435, no IMU). Record IR with the **emitter OFF** and
  fill per-unit calibration before use.

## Reproduce
```
podman run --rm --security-opt label=disable -e ROS_DOMAIN_ID=$((RANDOM%233)) \
  -v <datasets>:<datasets>:ro -v $PWD/scripts:/scripts:ro \
  -v <splatograph>:/splatograph:ro -v $PWD/output/tuning:/out \
  ghcr.io/bjoernellens1/splatograph-orbslam3:jazzy-orbbec-v3 \
  bash -lc "source /opt/ros/jazzy/setup.bash && bash /scripts/sweep.sh <slam_bag> /out/sweep 60 2"
```
