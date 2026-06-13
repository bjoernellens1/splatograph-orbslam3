#!/usr/bin/env python3
"""Build a rich provenance JSON for a produced bag.

Captures hardware + full configuration so a generated bag is self-describing:
camera model + intrinsics (from camera_info), SLAM mode + the complete config
YAML used, ORB-SLAM3 image tag, tooling git SHA, source bag, ROS_DOMAIN_ID, and
a UTC timestamp. Embedded as a latched /splatograph/provenance topic in the bag
by inject_poses.py (and written as a sidecar).

Usage: make_provenance.py --bag BAG --config CFG --hardware "Orbbec Femto Bolt"
         --slam-mode rgbd --image <tag> --git-sha <sha> --out prov.json
"""
import argparse
import datetime
import json
import os
import sys

from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def color_info(bag, topic="/camera/color/camera_info"):
    r = SequentialReader()
    r.open(StorageOptions(uri=bag, storage_id="mcap"), ConverterOptions("cdr", "cdr"))
    tmap = {t.name: t.type for t in r.get_all_topics_and_types()}
    if topic not in tmap:
        return None
    while r.has_next():
        n, d, _ = r.read_next()
        if n == topic:
            m = deserialize_message(d, get_message(tmap[topic]))
            K = m.k
            return {"width": m.width, "height": m.height,
                    "fx": K[0], "fy": K[4], "cx": K[2], "cy": K[5],
                    "distortion_model": m.distortion_model, "D": list(m.d),
                    "frame_id": m.header.frame_id}
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bag", required=True)
    ap.add_argument("--config", required=True)
    ap.add_argument("--hardware", required=True)
    ap.add_argument("--slam-mode", default="rgbd")
    ap.add_argument("--image", default=os.environ.get("ORBSLAM_IMAGE", ""))
    ap.add_argument("--git-sha", default=os.environ.get("GIT_SHA", ""))
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    prov = {
        "schema": "splatograph/provenance/v1",
        "produced_utc": datetime.datetime.now(datetime.timezone.utc).isoformat(),
        "hardware": {"camera": args.hardware, "camera_info": color_info(args.bag)},
        "slam": {
            "method": "ORB-SLAM3",
            "mode": args.slam_mode,
            "ros2_wrapper": "ros2_orb_slam3 (patched)",
            "image": args.image,
            "config_file": os.path.basename(args.config),
            "config_yaml": open(args.config).read(),
        },
        "tooling": {"splatograph_orbslam3_git_sha": args.git_sha},
        "source": {"bag": args.bag, "ros_domain_id": os.environ.get("ROS_DOMAIN_ID", "")},
        "pose_topic": "/camera_pose",
        "note": "Poses are ORB-SLAM3 estimates (Twc, frame_id=map). Reference quality, not GT.",
    }
    with open(args.out, "w") as f:
        json.dump(prov, f, indent=2)
    print(f"[provenance] wrote {args.out} (camera={args.hardware}, image={args.image}, sha={args.git_sha[:8]})",
          file=sys.stderr)


if __name__ == "__main__":
    main()
