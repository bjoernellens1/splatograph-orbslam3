#!/usr/bin/env python3
"""Decompress Orbbec RGB-D CompressedImage topics into raw sensor_msgs/Image.

The Femto bags publish:
  /camera/color/image_raw/compressed  CompressedImage  'rgb8; jpeg compressed bgr8'
  /camera/depth/image_raw/compressed  CompressedImage  '16UC1; png compressed'

ORB-SLAM3's rgbd_node_cpp wants raw Image on /camera/color/image_raw +
/camera/depth/image_raw, ApproximateTime-synced. This node decodes both via
cv2.imdecode (JPEG colour → bgr8; PNG → 16UC1, mm) and republishes raw, keeping
the original header (stamp + frame_id) so SLAM timestamps and the /camera_pose
reference stay aligned.

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
        # sync: pair colour+depth (ApproximateTime) and republish BOTH with the
        # colour stamp. The Femto colour/depth are hardware-synced (same capture)
        # but the bag stamps them ~100ms apart, which breaks RTAB-Map's approx
        # sync (mispaired frames -> wrong feature depth -> 0 inliers).
        self.sync = self.declare_parameter("sync", False).get_parameter_value().bool_value
        self.bridge = CvBridge()
        self.n_c = self.n_d = 0

        self.pub_color = self.create_publisher(Image, self.color_out, 10)
        self.pub_depth = self.create_publisher(Image, self.depth_out, 10)
        if self.sync:
            from message_filters import Subscriber, ApproximateTimeSynchronizer
            cs = Subscriber(self, CompressedImage, self.color_in, qos_profile=qos_profile_sensor_data)
            ds = Subscriber(self, CompressedImage, self.depth_in, qos_profile=qos_profile_sensor_data)
            self._ats = ApproximateTimeSynchronizer([cs, ds], queue_size=30, slop=0.15)
            self._ats.registerCallback(self._synced)
        else:
            self.create_subscription(CompressedImage, self.color_in, self._color, qos_profile_sensor_data)
            self.create_subscription(CompressedImage, self.depth_in, self._depth, qos_profile_sensor_data)
        self.get_logger().info(
            f"decompress(sync={self.sync}): {self.color_in}->{self.color_out} ({self.color_encoding}), "
            f"{self.depth_in}->{self.depth_out} (16UC1)")

    def _synced(self, cmsg: CompressedImage, dmsg: CompressedImage):
        """Decode colour+depth and publish both with the COLOUR stamp."""
        cimg = cv2.imdecode(np.frombuffer(cmsg.data, np.uint8), cv2.IMREAD_COLOR)
        dimg = cv2.imdecode(np.frombuffer(dmsg.data, np.uint8), cv2.IMREAD_UNCHANGED)
        if cimg is None or dimg is None:
            return
        if self.color_encoding == "rgb8":
            cimg = cv2.cvtColor(cimg, cv2.COLOR_BGR2RGB)
        if dimg.dtype != np.uint16:
            dimg = dimg.astype(np.uint16)
        co = self.bridge.cv2_to_imgmsg(cimg, encoding=self.color_encoding)
        do = self.bridge.cv2_to_imgmsg(dimg, encoding="16UC1")
        co.header = cmsg.header
        do.header = cmsg.header  # re-stamp depth to the colour stamp (same capture)
        self.pub_color.publish(co); self.pub_depth.publish(do)
        self.n_c += 1; self.n_d += 1

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
