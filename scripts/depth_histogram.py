#!/usr/bin/env python3
"""Dump a depth histogram (metres) from an Orbbec bag to set Stereo.ThDepth.

The effective ORB-SLAM3 close/far boundary is Stereo.b * Stereo.ThDepth; depth
points beyond it are treated monocular-style. We sample valid depth pixels and
report percentiles so ThDepth can be set to cover the scene's real depths.

Usage: depth_histogram.py <bag> [--n 60] [--factor 1000]
"""
import argparse
import numpy as np
import cv2
from rosbag2_py import SequentialReader, StorageOptions, ConverterOptions
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

ap = argparse.ArgumentParser()
ap.add_argument("bag")
ap.add_argument("--topic", default="/camera/depth/image_raw/compressed")
ap.add_argument("--n", type=int, default=60)
ap.add_argument("--factor", type=float, default=1000.0)
args = ap.parse_args()

reader = SequentialReader()
reader.open(StorageOptions(uri=args.bag, storage_id="mcap"), ConverterOptions("cdr", "cdr"))
tmap = {t.name: t.type for t in reader.get_all_topics_and_types()}
msgtype = get_message(tmap[args.topic])

vals = []
k = 0
while reader.has_next() and k < args.n:
    name, data, _ = reader.read_next()
    if name != args.topic:
        continue
    m = deserialize_message(data, msgtype)
    img = cv2.imdecode(np.frombuffer(m.data, np.uint8), cv2.IMREAD_UNCHANGED)
    if img is None:
        continue
    d = img.astype(np.float32) / args.factor
    d = d[(d > 0.05) & (d < 20.0)]
    if d.size:
        vals.append(d[::37])  # subsample
    k += 1

allv = np.concatenate(vals) if vals else np.array([0.0])
pcts = [5, 25, 50, 75, 90, 95, 99]
qs = np.percentile(allv, pcts)
print(f"depth(m) over {k} frames, {allv.size} samples: "
      + "  ".join(f"p{p}={q:.2f}" for p, q in zip(pcts, qs)))
print(f"valid-pixel fraction beyond 2.0m (current b*ThDepth) = {float((allv>2.0).mean()):.3f}")
for th in (40, 60, 80):
    print(f"  Stereo.ThDepth={th} (b=0.05 -> boundary {0.05*th:.1f}m): "
          f"frac depth WITHIN boundary = {float((allv <= 0.05*th).mean()):.3f}")
