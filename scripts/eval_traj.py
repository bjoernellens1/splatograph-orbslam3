#!/usr/bin/env python3
"""Evaluate an ORB-SLAM3 trajectory against the baked-in O3D reference.

Reads /slam/pose (estimate, Twc) and /camera_pose (reference, Twc) from a
recorded ros2 bag, associates by header timestamp, and reports the objective
tuple: (ATE-vs-O3D, tracking coverage %, track-loss gaps). ATE/RPE use Umeyama
SE(3) alignment via splatograph/utils/trajectory_eval.py (mount it + PYTHONPATH).

NOTE: the reference is O3D RGBD odometry, NOT ground truth — this measures
*agreement* with O3D, not absolute accuracy. ATE will not (and should not) be 0.

Usage: eval_traj.py <recorded_bag> [--est /slam/pose] [--ref /camera_pose]
                     [--max-dt 0.03] [--json out.json]
"""
import argparse
import json
import sys
import numpy as np

from utils.trajectory_eval import associate_by_timestamp, trajectory_metrics, _quat_to_rot

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def read_poses(bag, topic):
    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag, storage_id="mcap"), ConverterOptions("cdr", "cdr"))
    tmap = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in tmap:
        return np.array([]), np.empty((0, 4, 4))
    msgtype = get_message(tmap[topic])
    ts, c2w = [], []
    while reader.has_next():
        name, data, _ = reader.read_next()
        if name != topic:
            continue
        m = deserialize_message(data, msgtype)
        p = m.pose.position
        q = m.pose.orientation
        T = np.eye(4)
        T[:3, :3] = _quat_to_rot(np.array([q.w, q.x, q.y, q.z]))
        T[:3, 3] = [p.x, p.y, p.z]
        ts.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        c2w.append(T)
    if not ts:
        return np.array([]), np.empty((0, 4, 4))
    return np.asarray(ts, np.float64), np.stack(c2w)


def track_loss_gaps(ts_est, frame_period):
    """Count gaps in the estimate stream larger than 3x the frame period."""
    if len(ts_est) < 2:
        return 0, 0.0
    d = np.diff(np.sort(ts_est))
    thr = 3.0 * frame_period
    gaps = d[d > thr]
    return int(gaps.size), float(gaps.sum())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--est", default="/slam/pose")
    ap.add_argument("--ref", default="/camera_pose")
    ap.add_argument("--max-dt", type=float, default=0.03)
    ap.add_argument("--json", default="")
    ap.add_argument("--label", default="")
    args = ap.parse_args()

    ts_e, c2w_e = read_poses(args.bag, args.est)
    ts_r, c2w_r = read_poses(args.bag, args.ref)
    n_est, n_ref = len(ts_e), len(ts_r)

    frame_period = 1.0 / 30.0
    if n_ref > 1:
        frame_period = float(np.median(np.diff(np.sort(ts_r))))

    ts_m, em, rm, info = associate_by_timestamp(ts_e, c2w_e, ts_r, c2w_r, max_dt=args.max_dt)
    metrics = trajectory_metrics(em, rm) if len(em) else {
        "ate_rmse": None, "ate_mean": None, "rpe_trans_rmse": None,
        "rpe_rot_deg_rmse": None, "n_frames": 0}

    coverage = (n_est / n_ref) if n_ref else 0.0
    n_gaps, gap_secs = track_loss_gaps(ts_e, frame_period)

    out = {
        "label": args.label, "bag": args.bag,
        "n_est": n_est, "n_ref": n_ref,
        "coverage": round(coverage, 4),
        "n_matches": info.get("n_matches", 0),
        "track_loss_gaps": n_gaps, "track_loss_secs": round(gap_secs, 2),
        "ate_rmse_vs_o3d": metrics["ate_rmse"], "ate_mean_vs_o3d": metrics["ate_mean"],
        "rpe_trans_rmse": metrics["rpe_trans_rmse"], "rpe_rot_deg_rmse": metrics["rpe_rot_deg_rmse"],
    }
    print(json.dumps(out))
    ate = out["ate_rmse_vs_o3d"]
    ate_s = f"{ate:.4f}m" if ate is not None else "n/a"
    print(f"[eval] {args.label}  ATE-vs-O3D={ate_s}  coverage={coverage*100:.1f}%"
          f" ({n_est}/{n_ref})  track_loss_gaps={n_gaps} ({gap_secs:.1f}s)"
          f"  RPE_t={out['rpe_trans_rmse']}", file=sys.stderr)
    if args.json:
        with open(args.json, "w") as f:
            json.dump(out, f, indent=2)


if __name__ == "__main__":
    main()
