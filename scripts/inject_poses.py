#!/usr/bin/env python3
"""Offline-merge ORB-SLAM3 poses into a copy of a pose-less bag.

Writes a NEW mcap = every original message verbatim (topics, types, timestamps
preserved exactly) + /camera_pose (the recorded /slam/pose, renamed) + a latched
/splatograph/provenance (std_msgs/String JSON: hardware, full SLAM config + params,
tooling git SHAs, source bag, domain, timestamp). This is how the *_slam bags are
re-created from raw bags, but with rich, self-contained provenance.

Original messages are copied as raw serialized bytes (lossless, no re-stamp).
/slam/pose (PoseStamped) bytes are type-identical to /camera_pose, so they are
re-written under the new topic name without deserialization.

Usage:
  inject_poses.py --orig RAW_BAG --poses POSES_BAG --out NEW_BAG
                  --provenance PROV_JSON [--pose-in /slam/pose] [--pose-out /camera_pose]
"""
import argparse
import json
import os
import sys

from rosbag2_py import (SequentialReader, SequentialWriter, StorageOptions,
                        ConverterOptions, TopicMetadata)
from rclpy.serialization import serialize_message
from std_msgs.msg import String


def topics_of(bag):
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id="mcap"), ConverterOptions("cdr", "cdr"))
    return {t.name: t for t in r.get_all_topics_and_types()}, r


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--orig", required=True)
    ap.add_argument("--poses", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--provenance", required=True, help="path to provenance JSON to embed")
    ap.add_argument("--pose-in", default="/slam/pose")
    ap.add_argument("--pose-out", default="/camera_pose")
    args = ap.parse_args()

    if os.path.exists(args.out):
        sys.exit(f"output {args.out} exists; remove first")
    prov = json.load(open(args.provenance))

    orig_topics, orig_reader = topics_of(args.orig)
    pose_topics, _ = topics_of(args.poses)
    if args.pose_in not in pose_topics:
        sys.exit(f"{args.pose_in} not in poses bag {args.poses}")
    pose_type = pose_topics[args.pose_in].type  # geometry_msgs/msg/PoseStamped

    writer = SequentialWriter()
    writer.open(StorageOptions(uri=args.out, storage_id="mcap"), ConverterOptions("cdr", "cdr"))

    # Register all original topics verbatim (preserve QoS + type hash) + the
    # injected pose + provenance. Jazzy TopicMetadata needs an explicit id.
    ptm = pose_topics[args.pose_in]
    next_id = 0
    for name, tm in orig_topics.items():
        writer.create_topic(tm)  # reader's TopicMetadata: keeps id/qos/hash
        next_id = max(next_id, tm.id + 1)
    if args.pose_out not in orig_topics:
        writer.create_topic(TopicMetadata(
            id=next_id, name=args.pose_out, type=pose_type, serialization_format="cdr",
            offered_qos_profiles=ptm.offered_qos_profiles,
            type_description_hash=ptm.type_description_hash))
        next_id += 1
    writer.create_topic(TopicMetadata(
        id=next_id, name="/splatograph/provenance", type="std_msgs/msg/String",
        serialization_format="cdr"))

    # Provenance message (write at the bag's first timestamp once known).
    prov_msg = String(); prov_msg.data = json.dumps(prov, indent=2)
    prov_bytes = serialize_message(prov_msg)

    # 1. copy original messages verbatim (lossless); index image header.stamp ->
    #    bag-time via /camera/color/camera_info (shares image stamps, cheap to
    #    deserialize) so injected poses land at the SAME bag-time as their image.
    from rclpy.serialization import deserialize_message as _de
    from rosidl_runtime_py.utilities import get_message as _gm
    ci_topic = "/camera/color/camera_info"
    ci_type = orig_topics[ci_topic].type if ci_topic in orig_topics else None
    stamp_to_bagt = {}  # header.stamp_ns -> bag-time ns
    t0 = None
    n_orig = 0
    while orig_reader.has_next():
        name, data, t = orig_reader.read_next()
        if t0 is None:
            t0 = t
            writer.write("/splatograph/provenance", prov_bytes, t0)
        writer.write(name, data, t)
        n_orig += 1
        if ci_type and name == ci_topic:
            m = _de(data, _gm(ci_type))
            stamp_to_bagt[m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec] = t

    keys = sorted(stamp_to_bagt) if stamp_to_bagt else []
    import bisect
    def aligned_bagt(stamp_ns):
        if not keys:
            return stamp_ns  # fallback: use the sensor stamp directly
        i = bisect.bisect_left(keys, stamp_ns)
        cands = [k for k in (i, i - 1) if 0 <= k < len(keys)]
        best = min(cands, key=lambda j: abs(keys[j] - stamp_ns))
        return stamp_to_bagt[keys[best]]

    # 2. inject /slam/pose -> /camera_pose at the matched image bag-time (bytes
    #    are type-identical; only the bag-time is realigned).
    pose_msgtype = _gm(pose_type)
    pr = SequentialReader()
    pr.open(StorageOptions(uri=args.poses, storage_id="mcap"), ConverterOptions("cdr", "cdr"))
    n_pose = 0
    while pr.has_next():
        name, data, t = pr.read_next()
        if name != args.pose_in:
            continue
        pm = _de(data, pose_msgtype)
        stamp_ns = pm.header.stamp.sec * 1_000_000_000 + pm.header.stamp.nanosec
        writer.write(args.pose_out, data, aligned_bagt(stamp_ns))
        n_pose += 1

    print(f"[inject] {args.out}: {n_orig} original msgs + {n_pose} {args.pose_out} "
          f"+ provenance ({len(orig_topics)} orig topics)")


if __name__ == "__main__":
    main()
