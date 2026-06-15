# Running ORB-SLAM3 nicely on Orbbec RGB-D — setup & tuned parameters

Operational checklist for getting ORB-SLAM3 RGB-D tracking *well* on Orbbec Femto
(Bolt/Mega) bags. For the full parameter sweep and how the numbers were derived,
see [`2026-06-13-orbbec-rgbd-tuning.md`](2026-06-13-orbbec-rgbd-tuning.md); this
note is the "what you actually have to do" summary.

## 1. The pipeline (what feeds ORB-SLAM3)

Orbbec bags publish **compressed** RGB-D, but `rgbd_node_cpp` wants raw, synced
`sensor_msgs/Image`. So the chain is:

```
ros2 bag play
  → decompress_rgbd_node.py     # JPEG colour → bgr8, PNG → 16UC1 depth (raw Image)
  → rgbd_node_cpp               # per-bag config (see §3)
  → record /slam/pose vs /camera_pose
  → eval_traj.py                # ATE/RPE, Umeyama SE(3) aligned
```

`run_orbslam_rgbd.sh` wires this up. The decompressor **keeps the original header
(stamp + frame_id)** so SLAM timestamps stay aligned with the `/camera_pose`
reference — don't restamp.

## 2. Three data checks the Orbbec publisher doesn't declare

1. **Depth ↔ colour are pre-aligned** — same 1280×720, same `frame_id`
   (`camera_color_optical_frame`), same K. Feed depth directly, **no reprojection**.
2. **Images are raw/distorted** — `camera_info` has `P==K, R==I, D!=0`, so the
   distortion coeffs (`k1,k2,p1,p2,k3`) must be applied. (Effect on *trajectory*
   ATE is ~sub-mm here, but it's correct and matters for rendering.)
3. **Depth is 16-bit unsigned millimetres** ⇒ `RGBD.DepthMapFactor: 1000.0`.
   (Contrast: the ros2-converted **TUM** bags are 32FC1 *metres* → factor `1.0`.
   Always verify with `ros2 topic echo <depth> --field encoding --once`.)

## 3. Tuned RGB-D profile (`config/OrbbecFemtoBolt_RGBD.yaml`)

Intrinsics are **per-unit** (factory calibration) — regenerate from the bag's
`/camera/color/camera_info` with `scripts/make_orbbec_config.py`. The ORB/depth
**profile** below is portable across Bolt/Mega (shared optics):

| Param | Value | Why |
|---|---|---|
| `RGBD.DepthMapFactor` | **1000.0** | depth is 16UC1 mm |
| `Stereo.b` | 0.0500 | virtual baseline; close/far = `b·ThDepth` |
| `Stereo.ThDepth` | **60.0** | ⇒ 3.0 m boundary. Tuned: covers reliable mid-range Femto depth, **excludes noisy 3–4 m far returns that *hurt* ATE**. 40 wasted 87% of depth; 80/100 added noise. |
| `ORBextractor.nFeatures` | **1250** | more did not help |
| `Camera.RGB` | **0** | decompressor publishes bgr8 |
| scaleFactor / nLevels / FAST | 1.2 / 8 / 20,7 | stock |

Lead tuning axis was **ThDepth**, not features. Run-to-run variance ≈ 2 mm.

## 4. The pose-convention trap (silent, costly)

ORB-SLAM3 ROS2 wrappers can export **either** `Twc` (camera→world, **correct** for
`PoseStamped(frame_id="map")`) **or** `Tcw` (world→camera). Exporting `Tcw` as if
it were `Twc` silently corrupts everything downstream (95% dead Gaussians, blobs).

**Diagnostic** — camera z-axis vs motion direction:
`alignment = mean(dot(Δpos/|Δpos|, R[:,2]))`. Handheld c2w ⇒ **+0.1…+0.5**; a
mis-exported w2c ⇒ a consistent **≈ −0.92**. If you see negative, either
regenerate the bag or invert in the bridge:
`inv = [[Rᵀ, −Rᵀ t],[0,1]]`. Check this immediately for any new bag.

## 5. Runtime facts

- **`-r 1.0` is real-time**: 29.9 pose/s = full 30 fps, 100% coverage, 0 losses,
  same ATE as `-r 0.5`. Don't slow playback.
- **Vocabulary**: use the `.bin` (`ORBvoc.txt.bin`) — loads in ~0.3 s vs ~15 s for
  the text form. Wait on the `node ready` log line, not a fixed sleep.
- Randomized `ROS_DOMAIN_ID=$((RANDOM%233))`, one GPU run at a time, `--init`.

## 6. Results (ATE vs the O3D `/camera_pose` reference — *agreement*, not GT)

| bag | ATE | RPE_t | coverage |
|---|---|---|---|
| workshop1_slam (full) | 235 mm | 8 mm | 100% |
| kitchen1_slam (full) | 43 mm | — | 100% |
| table1_slam (full) | **37 mm** | 4 mm | 100% |

The high workshop1 ATE with tiny RPE is **not** a regression: the trajectories
agree locally (RPE 8 mm) and drift apart globally — the O3D reference is itself
drifting odometry without loop closure, so ORB-SLAM3 (with BA + loop closure) can
be *more* correct yet disagree more. For absolute accuracy use TUM mocap GT (see
`run_orbslam_tum.sh` + `eval_tum.py`).
