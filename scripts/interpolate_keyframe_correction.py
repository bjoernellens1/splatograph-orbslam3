#!/usr/bin/env python3
"""Fallback for reconstructing a FINAL (loop-closed) per-frame trajectory
when only SaveKeyFrameTrajectoryTUM (keyframes only) is available, not
SaveTrajectoryTUM (every frame) -- e.g. an ORB-SLAM3 image that has not been
rebuilt with the extra Shutdown()-time call.

Idea: at shutdown, ORB-SLAM3 corrects each KEYFRAME's pose via global BA /
pose-graph optimisation (loop closure). The live per-frame stream recorded
during playback (/slam/pose, one message per tracked frame -- including every
keyframe, at the moment it was created) never receives those corrections; it
is the CAUSAL estimate. For every keyframe we therefore have two poses of the
SAME frame: T_kf_causal (from the live stream, uncorrected) and T_kf_final
(from KeyFrameTrajectory.txt, corrected). The SE(3) delta

    D_i = T_kf_final,i @ inv(T_kf_causal,i)

is the correction loop closure/BA applied at that keyframe. For a non-
keyframe causal pose T_causal(t) between two keyframes i (t_i <= t) and
j=i+1 (t_j >= t), we don't know its true correction, so we approximate it by
blending the two enclosing keyframes' corrections:

    D(t)      = slerp(D_i.rot, D_j.rot, a) with lerp'd translation, a in [0,1]
    a         = (t - t_i) / (t_j - t_i)
    T_final(t) = D(t) @ T_causal(t)

This is an approximation, NOT the same thing SaveTrajectoryTUM would produce
(that composes each frame's stored relative-to-reference-keyframe transform
with the reference keyframe's own corrected pose -- a per-frame, not
temporal-neighbour, correction). Prefer a rebuilt image with SaveTrajectoryTUM
when available; use this script as a fallback or a cross-check.

Frame convention: KeyFrameTrajectory.txt is written by ORB-SLAM3 in its own
native camera convention (OpenCV axes); a live-recorded /slam/pose bag is
already rotated into ROS REP-103 by the patched node (see
patches/apply_patch.py). Deltas MUST be computed in a single consistent
frame -- this script rotates the keyframe-final txt into ROS convention
(same B matrix as tum_trajectory_to_pose_bag.py) before computing anything,
unless --keyframes-convention orbslam_native says it is already ROS-frame.

Usage:
  interpolate_keyframe_correction.py \\
      --keyframes-final KeyFrameTrajectory.txt \\
      --causal-bag /out/floor3_final_poses --causal-topic /slam/pose \\
      --out /out/floor3_causal_corrected_TUM.txt \\
      [--keyframes-convention ros_rep103] [--match-tol 0.002]

Output is a TUM file in ROS REP-103 convention (ready for
tum_trajectory_to_pose_bag.py --convention orbslam_native, i.e. no further
rotation needed).
"""
import argparse
import bisect
import json
import sys

import numpy as np

from tum_trajectory_to_pose_bag import (
    B_ROS_REP103, read_tum, quat_to_rot, rot_to_quat, apply_ros_rep103,
)


def read_causal_bag(bag_uri, topic):
    from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = SequentialReader()
    reader.open(StorageOptions(uri=bag_uri, storage_id="mcap"), ConverterOptions("cdr", "cdr"))
    tmap = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in tmap:
        sys.exit(f"{topic} not found in {bag_uri}; topics: {sorted(tmap)}")
    msgtype = get_message(tmap[topic])
    ts, txyz, qxyzw = [], [], []
    while reader.has_next():
        name, data, _ = reader.read_next()
        if name != topic:
            continue
        m = deserialize_message(data, msgtype)
        ts.append(m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)
        p = m.pose.position
        q = m.pose.orientation
        txyz.append((p.x, p.y, p.z))
        qxyzw.append((q.x, q.y, q.z, q.w))
    if not ts:
        sys.exit(f"no messages on {topic} in {bag_uri}")
    order = np.argsort(ts)
    return (np.asarray(ts, np.float64)[order], np.asarray(txyz, np.float64)[order],
            np.asarray(qxyzw, np.float64)[order])


def se3(t, R):
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = t
    return T


def slerp(q0, q1, a):
    """Shortest-path slerp, q's as [x,y,z,w], a in [0,1]."""
    d = float(np.dot(q0, q1))
    if d < 0.0:
        q1 = -q1
        d = -d
    d = min(1.0, d)
    if d > 0.9995:
        out = q0 + a * (q1 - q0)
        return out / np.linalg.norm(out)
    theta0 = np.arccos(d)
    theta = theta0 * a
    q2 = q1 - q0 * d
    q2 = q2 / np.linalg.norm(q2)
    return q0 * np.cos(theta) + q2 * np.sin(theta)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--keyframes-final", required=True, help="KeyFrameTrajectory.txt (TUM, post loop-closure)")
    ap.add_argument("--causal-bag", required=True, help="bag with the live per-frame /slam/pose stream")
    ap.add_argument("--causal-topic", default="/slam/pose")
    ap.add_argument("--out", required=True, help="output TUM file, ROS-frame, per-frame")
    ap.add_argument("--keyframes-convention", choices=["ros_rep103", "orbslam_native"], default="ros_rep103",
                     help="orbslam_native means KeyFrameTrajectory.txt still needs the B rotation "
                          "applied (the normal case -- ORB-SLAM3 writes its own native frame). "
                          "ros_rep103 means it is already rotated (skip).")
    ap.add_argument("--match-tol", type=float, default=0.002,
                     help="max |dt| (s) to match a KeyFrameTrajectory.txt row to a causal-bag row")
    ap.add_argument("--report-json", default=None)
    args = ap.parse_args()

    kf_ts, kf_t, kf_q = read_tum(args.keyframes_final)
    if args.keyframes_convention == "ros_rep103":
        # already ROS frame -- no rotation
        kf_t_ros, kf_R_ros = kf_t, quat_to_rot(kf_q)
    else:
        kf_R = quat_to_rot(kf_q)
        kf_t_ros, kf_R_ros = apply_ros_rep103(kf_t, kf_R)

    c_ts, c_t, c_q = read_causal_bag(args.causal_bag, args.causal_topic)
    c_R = quat_to_rot(c_q)

    # match each keyframe-final row to the nearest causal row by timestamp
    matched = []  # (t_i, D_rot(3,3), D_trans(3,))
    n_unmatched = 0
    for i in range(len(kf_ts)):
        j = bisect.bisect_left(c_ts, kf_ts[i])
        cands = [k for k in (j - 1, j) if 0 <= k < len(c_ts)]
        if not cands:
            n_unmatched += 1
            continue
        best = min(cands, key=lambda k: abs(c_ts[k] - kf_ts[i]))
        if abs(c_ts[best] - kf_ts[i]) > args.match_tol:
            n_unmatched += 1
            continue
        T_final = se3(kf_t_ros[i], kf_R_ros[i])
        T_causal = se3(c_t[best], c_R[best])
        D = T_final @ np.linalg.inv(T_causal)
        matched.append((kf_ts[i], D[:3, :3], D[:3, 3]))

    if len(matched) < 2:
        sys.exit(f"only {len(matched)} keyframes matched (tol={args.match_tol}s) -- "
                 f"cannot interpolate; check --keyframes-convention and timestamps")

    m_ts = np.array([m[0] for m in matched])
    m_R = np.stack([m[1] for m in matched])
    m_t = np.stack([m[2] for m in matched])
    m_q = rot_to_quat(m_R)

    n_interp = n_extrap_before = n_extrap_after = 0
    out_t = np.empty_like(c_t)
    out_R = np.empty_like(c_R)
    for k in range(len(c_ts)):
        t = c_ts[k]
        idx = bisect.bisect_left(m_ts, t)
        if idx <= 0:
            Di_R, Di_t = m_R[0], m_t[0]
            n_extrap_before += 1
        elif idx >= len(m_ts):
            Di_R, Di_t = m_R[-1], m_t[-1]
            n_extrap_after += 1
        else:
            t0, t1 = m_ts[idx - 1], m_ts[idx]
            a = 0.0 if t1 <= t0 else (t - t0) / (t1 - t0)
            a = min(1.0, max(0.0, a))
            q_blend = slerp(m_q[idx - 1], m_q[idx], a)
            Di_R = quat_to_rot(q_blend[None, :])[0]
            Di_t = (1 - a) * m_t[idx - 1] + a * m_t[idx]
            n_interp += 1
        D = se3(Di_t, Di_R)
        T_c = se3(c_t[k], c_R[k])
        T_f = D @ T_c
        out_R[k] = T_f[:3, :3]
        out_t[k] = T_f[:3, 3]

    out_q = rot_to_quat(out_R)
    with open(args.out, "w") as f:
        for k in range(len(c_ts)):
            f.write(f"{c_ts[k]:.6f} {out_t[k,0]:.9f} {out_t[k,1]:.9f} {out_t[k,2]:.9f} "
                    f"{out_q[k,0]:.9f} {out_q[k,1]:.9f} {out_q[k,2]:.9f} {out_q[k,3]:.9f}\n")

    report = {
        "keyframes_final_rows": int(len(kf_ts)),
        "keyframes_matched": int(len(matched)),
        "keyframes_unmatched": int(n_unmatched),
        "causal_frames": int(len(c_ts)),
        "frames_interpolated": int(n_interp),
        "frames_extrapolated_before_first_kf": int(n_extrap_before),
        "frames_extrapolated_after_last_kf": int(n_extrap_after),
        "match_tol_s": args.match_tol,
    }
    print("[interpolate_keyframe_correction]", json.dumps(report))
    if args.report_json:
        json.dump(report, open(args.report_json, "w"), indent=2)


if __name__ == "__main__":
    main()
