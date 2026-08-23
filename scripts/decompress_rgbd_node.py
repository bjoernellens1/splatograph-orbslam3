#!/usr/bin/env python3
"""Decompress Orbbec RGB-D CompressedImage topics into raw sensor_msgs/Image.

The Femto bags publish:
  /camera/color/image_raw/compressed  CompressedImage  'rgb8; jpeg compressed bgr8'
  /camera/depth/image_raw/compressed  CompressedImage  '16UC1; png compressed'

This node is DECODE-ONLY (splatograph-rgbd-sync#1 canonical topology,
2026-08-23 migration, splatograph-orbslam3#3): it decodes each stream
independently via cv2.imdecode (JPEG colour -> bgr8; PNG -> 16UC1, mm) and
republishes raw with the original per-stream header (stamp + frame_id)
untouched. It does no pairing and no re-stamping -- that is
splatograph_rgbd_sync's SyncNode's job, and only its job (see that repo's
README: "the decoder is decode-only ... Downstream mapping must never split
an authoritative pair and nearest-neighbor re-pair it"). SyncNode subscribes
to this node's raw output and publishes the single authoritative
RGBDSynced message that ORB-SLAM3's rgbd_node_cpp/rgbd_inertial_node_cpp now
consume directly.

Previously (pre-migration) this node had an opt-in `sync` parameter that
ApproximateTime-paired and re-stamped color+depth itself, as a workaround for
the Femto bag's ~100ms baked-in stamp offset between the two streams. That
was exactly the downstream/software re-sync pattern splatograph's CLAUDE.md
HARD CONSTRAINT prohibits (color/depth pairing must be solved once, upstream,
never re-paired downstream with a sliding window) and duplicated work
SyncNode now owns authoritatively -- removed.

Params (--ros-args -p ...):
  color_in   (default /camera/color/image_raw/compressed)
  depth_in   (default /camera/depth/image_raw/compressed)
  color_out  (default /camera/color/image_raw)
  depth_out  (default /camera/depth/image_raw)
  color_encoding (default bgr8)   # publish encoding for colour
"""
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from cv_bridge import CvBridge


class DecompressRGBD(Node):
    def __init__(self):
        super().__init__("decompress_rgbd")
        g = lambda n, d: self.declare_parameter(n, d).get_parameter_value().string_value
        self.color_in = g("color_in", "/camera/color/image_raw/compressed")
        self.depth_in = g("depth_in", "/camera/depth/image_raw/compressed")
        self.color_out = g("color_out", "/camera/color/image_raw")
        self.depth_out = g("depth_out", "/camera/depth/image_raw")
        self.color_encoding = g("color_encoding", "bgr8")
        self.bridge = CvBridge()
        self.n_c = self.n_d = 0

        self.pub_color = self.create_publisher(Image, self.color_out, 10)
        self.pub_depth = self.create_publisher(Image, self.depth_out, 10)
        self.create_subscription(CompressedImage, self.color_in, self._color, qos_profile_sensor_data)
        self.create_subscription(CompressedImage, self.depth_in, self._depth, qos_profile_sensor_data)
        self.get_logger().info(
            f"decompress (decode-only): {self.color_in}->{self.color_out} ({self.color_encoding}), "
            f"{self.depth_in}->{self.depth_out} (16UC1); pairing owned downstream by "
            f"splatograph_rgbd_sync's SyncNode")

    def _color(self, msg: CompressedImage):
        buf = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(buf, cv2.IMREAD_COLOR)  # bgr8
        if img is None:
            return
        if self.color_encoding == "rgb8":
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        out = self.bridge.cv2_to_imgmsg(img, encoding=self.color_encoding)
        out.header = msg.header
        self.pub_color.publish(out)
        self.n_c += 1

    def _depth(self, msg: CompressedImage):
        buf = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(buf, cv2.IMREAD_UNCHANGED)  # uint16, mm
        if img is None:
            return
        if img.dtype != np.uint16:
            img = img.astype(np.uint16)
        out = self.bridge.cv2_to_imgmsg(img, encoding="16UC1")
        out.header = msg.header
        self.pub_depth.publish(out)
        self.n_d += 1


def main():
    rclpy.init()
    node = DecompressRGBD()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f"decompress done: color={node.n_c} depth={node.n_d}")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
