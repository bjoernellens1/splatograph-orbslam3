#!/usr/bin/env python3
"""Convert a TUM-format ORB-SLAM3 trajectory file into a ROS2 mcap bag with
geometry_msgs/PoseStamped on /slam/pose, so it can be merged into a raw bag
by inject_poses.py exactly like a live-recorded /slam/pose bag.

Why this exists: ORB_SLAM3::System::SaveTrajectoryTUM("CameraTrajectory.txt")
(and SaveKeyFrameTrajectoryTUM) write the FINAL, loop-closed, globally
consistent trajectory to a plain text file at shutdown -- not to a ROS topic.
To compare that trajectory against the CAUSAL live /camera_pose stream (same
convention, same bag machinery, same downstream eval/inject tooling) it needs
to become a ROS2 bag topic the same shape as /slam/pose.

TUM format: one pose per line, whitespace separated
    timestamp tx ty tz qx qy qz qw
timestamp is seconds (float, ORB-SLAM3 writes setprecision(6) -> ~1us
resolution), tx/ty/tz in metres, qx..qw a unit quaternion. Twc (camera-in-
world), ORB-SLAM3's own convention (Z forward, Y down -- OpenCV/computer-
vision axes), NOT ROS REP-103 (X forward, Z up).

Convention handling (--convention):
  orbslam_native  -- write poses exactly as read from the txt (no rotation).
                      Use this if the input is already ROS-frame (e.g. produced
                      by a script that already applied the B matrix).
  ros_rep103      -- (default) apply the SAME fixed rotation the patched
                      rgbd_node_cpp / rgbd_inertial_node_cpp apply to every
                      live /slam/pose sample before publishing (see
                      patches/apply_patch.py, RGBD_NODE_CPP / _POSE_PUB):
                          B = [[0,0,1],[-1,0,0],[0,-1,0]]
                          t_ros = B @ t
                          R_ros = B @ R @ B.T
                      This MUST match the causal bag's convention or the two
                      trajectories are not comparable (a missing/extra B
                      rotates the whole trajectory ~90 degrees while leaving
                      frame count, sync %, and "0 jumps" checks all green --
                      those checks do not catch a frame convention bug).

Timestamps: TUM timestamps are written from the same header.stamp used to
call TrackRGBD in the live node (see rgbd_example.cpp: `t =
msgRGB->header.stamp.sec + msgRGB->header.stamp.nanosec*1e-9`), so they are
directly the raw bag's image capture epoch stamps -- no offset/rebase needed.
The stamp is round-tripped as sec/nanosec (round to the nearest ns) so
inject_poses.py's image-stamp alignment (which matches on header.stamp, not
bag-time) works identically to a live-recorded /slam/pose bag.

Usage:
  tum_trajectory_to_pose_bag.py --tum CameraTrajectory.txt --out /out/final_poses \
      [--convention ros_rep103] [--pose-topic /slam/pose] [--frame-id map]
"""
import argparse
import sys

import numpy as np


B_ROS_REP103 = np.array([
    [0.0, 0.0, 1.0],
    [-1.0, 0.0, 0.0],
    [0.0, -1.0, 0.0],
])


def read_tum(path):
    """Return (N,) timestamps float64, (N,3) t, (N,4) quat[x,y,z,w]."""
    ts, txyz, qxyzw = [], [], []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) != 8:
                raise SystemExit(f"malformed TUM line (expected 8 fields, got {len(parts)}): {line!r}")
            t, tx, ty, tz, qx, qy, qz, qw = (float(x) for x in parts)
            ts.append(t)
            txyz.append((tx, ty, tz))
            qxyzw.append((qx, qy, qz, qw))
    if not ts:
        raise SystemExit(f"no poses read from {path}")
    return np.asarray(ts, np.float64), np.asarray(txyz, np.float64), np.asarray(qxyzw, np.float64)


def quat_to_rot(qxyzw):
    """(N,4) [x,y,z,w] -> (N,3,3) rotation matrices."""
    x, y, z, w = qxyzw[:, 0], qxyzw[:, 1], qxyzw[:, 2], qxyzw[:, 3]
    n = np.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    N = qxyzw.shape[0]
    R = np.empty((N, 3, 3), np.float64)
    R[:, 0, 0] = 1 - 2 * (y * y + z * z)
    R[:, 0, 1] = 2 * (x * y - z * w)
    R[:, 0, 2] = 2 * (x * z + y * w)
    R[:, 1, 0] = 2 * (x * y + z * w)
    R[:, 1, 1] = 1 - 2 * (x * x + z * z)
    R[:, 1, 2] = 2 * (y * z - x * w)
    R[:, 2, 0] = 2 * (x * z - y * w)
    R[:, 2, 1] = 2 * (y * z + x * w)
    R[:, 2, 2] = 1 - 2 * (x * x + y * y)
    return R


def rot_to_quat(R):
    """(N,3,3) -> (N,4) [x,y,z,w], via a numerically-stable per-row branch."""
    N = R.shape[0]
    q = np.empty((N, 4), np.float64)
    for i in range(N):
        m = R[i]
        tr = m[0, 0] + m[1, 1] + m[2, 2]
        if tr > 0:
            s = 0.5 / np.sqrt(tr + 1.0)
            w = 0.25 / s
            x = (m[2, 1] - m[1, 2]) * s
            y = (m[0, 2] - m[2, 0]) * s
            z = (m[1, 0] - m[0, 1]) * s
        elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
            s = 2.0 * np.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
            w = (m[2, 1] - m[1, 2]) / s
            x = 0.25 * s
            y = (m[0, 1] + m[1, 0]) / s
            z = (m[0, 2] + m[2, 0]) / s
        elif m[1, 1] > m[2, 2]:
            s = 2.0 * np.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
            w = (m[0, 2] - m[2, 0]) / s
            x = (m[0, 1] + m[1, 0]) / s
            y = 0.25 * s
            z = (m[1, 2] + m[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
            w = (m[1, 0] - m[0, 1]) / s
            x = (m[0, 2] + m[2, 0]) / s
            y = (m[1, 2] + m[2, 1]) / s
            z = 0.25 * s
        q[i] = (x, y, z, w)
    n = np.linalg.norm(q, axis=1, keepdims=True)
    return q / n


def apply_ros_rep103(t, R):
    """t_ros = B @ t ; R_ros = B @ R @ B.T  (matches rgbd_example.cpp)."""
    B = B_ROS_REP103
    t_ros = t @ B.T
    R_ros = np.einsum("ij,njk,lk->nil", B, R, B)
    return t_ros, R_ros


def write_pose_bag(out_uri, ts, t, quat_xyzw, pose_topic, frame_id):
    from rosbag2_py import SequentialWriter, StorageOptions, ConverterOptions, TopicMetadata
    from rclpy.serialization import serialize_message
    from geometry_msgs.msg import PoseStamped

    writer = SequentialWriter()
    writer.open(StorageOptions(uri=out_uri, storage_id="mcap"), ConverterOptions("cdr", "cdr"))
    writer.create_topic(TopicMetadata(
        id=0, name=pose_topic, type="geometry_msgs/msg/PoseStamped", serialization_format="cdr"))

    n = 0
    for i in range(len(ts)):
        sec = int(np.floor(ts[i]))
        nanosec = int(round((ts[i] - sec) * 1e9))
        if nanosec >= 1_000_000_000:
            sec += 1
            nanosec -= 1_000_000_000
        msg = PoseStamped()
        msg.header.stamp.sec = sec
        msg.header.stamp.nanosec = nanosec
        msg.header.frame_id = frame_id
        msg.pose.position.x = float(t[i, 0])
        msg.pose.position.y = float(t[i, 1])
        msg.pose.position.z = float(t[i, 2])
        msg.pose.orientation.x = float(quat_xyzw[i, 0])
        msg.pose.orientation.y = float(quat_xyzw[i, 1])
        msg.pose.orientation.z = float(quat_xyzw[i, 2])
        msg.pose.orientation.w = float(quat_xyzw[i, 3])
        bag_time_ns = sec * 1_000_000_000 + nanosec
        writer.write(pose_topic, serialize_message(msg), bag_time_ns)
        n += 1
    return n


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tum", required=True, help="TUM trajectory txt (timestamp tx ty tz qx qy qz qw)")
    ap.add_argument("--out", required=True, help="output bag directory (mcap)")
    ap.add_argument("--convention", choices=["ros_rep103", "orbslam_native"], default="ros_rep103",
                     help="ros_rep103 (default): apply the same B-rotation the live node applies. "
                          "orbslam_native: write the txt poses unrotated (already-ROS-frame input).")
    ap.add_argument("--pose-topic", default="/slam/pose")
    ap.add_argument("--frame-id", default="map")
    args = ap.parse_args()

    ts, t, quat_xyzw = read_tum(args.tum)
    if args.convention == "ros_rep103":
        R = quat_to_rot(quat_xyzw)
        t_out, R_out = apply_ros_rep103(t, R)
        quat_out = rot_to_quat(R_out)
    else:
        t_out, quat_out = t, quat_xyzw

    import os
    if os.path.exists(args.out):
        sys.exit(f"output {args.out} exists; remove first")
    n = write_pose_bag(args.out, ts, t_out, quat_out, args.pose_topic, args.frame_id)
    print(f"[tum_trajectory_to_pose_bag] {args.tum} -> {args.out}: {n} poses on {args.pose_topic} "
          f"(convention={args.convention}, t range [{ts[0]:.3f}, {ts[-1]:.3f}])")


if __name__ == "__main__":
    main()
