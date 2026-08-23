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


def _stamp(msg) -> float:
    return float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9


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
        # Issue #865 stage-bisection instrumentation: distinguish "the message
        # never arrived at this node" from "it arrived and we dropped/failed to
        # decode it". `arrived_*` counts callback entries; `n_c`/`n_d` count
        # successful publishes; `decode_fail_*` is the difference's only cause.
        self.arrived_c = self.arrived_d = 0
        self.decode_fail_c = self.decode_fail_d = 0
        self.last_c_stamp = self.last_d_stamp = 0.0
        self.report_period = self.declare_parameter(
            "report_period_sec", 0.0).get_parameter_value().double_value

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
        if self.report_period > 0.0:
            self.create_timer(self.report_period, self.report_counts)

    def report_counts(self, final=False):
        self.get_logger().info(
            f"{'FINAL ' if final else ''}LEDGER color arrived={self.arrived_c} "
            f"published={self.n_c} decode_fail={self.decode_fail_c} "
            f"last_stamp={self.last_c_stamp:.6f} | depth arrived={self.arrived_d} "
            f"published={self.n_d} decode_fail={self.decode_fail_d} "
            f"last_stamp={self.last_d_stamp:.6f}")

    def _synced(self, cmsg: CompressedImage, dmsg: CompressedImage):
        """Decode colour+depth and publish both with the COLOUR stamp."""
        self.arrived_c += 1
        self.arrived_d += 1
        self.last_c_stamp = _stamp(cmsg)
        self.last_d_stamp = _stamp(dmsg)
        cimg = cv2.imdecode(np.frombuffer(cmsg.data, np.uint8), cv2.IMREAD_COLOR)
        dimg = cv2.imdecode(np.frombuffer(dmsg.data, np.uint8), cv2.IMREAD_UNCHANGED)
        if cimg is None or dimg is None:
            self.decode_fail_c += 1
            self.decode_fail_d += 1
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
        self.arrived_c += 1
        self.last_c_stamp = _stamp(msg)
        buf = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(buf, cv2.IMREAD_COLOR)  # bgr8
        if img is None:
            self.decode_fail_c += 1
            return
        if self.color_encoding == "rgb8":
            img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        out = self.bridge.cv2_to_imgmsg(img, encoding=self.color_encoding)
        out.header = msg.header
        self.pub_color.publish(out)
        self.n_c += 1

    def _depth(self, msg: CompressedImage):
        self.arrived_d += 1
        self.last_d_stamp = _stamp(msg)
        buf = np.frombuffer(msg.data, np.uint8)
        img = cv2.imdecode(buf, cv2.IMREAD_UNCHANGED)  # uint16, mm
        if img is None:
            self.decode_fail_d += 1
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
    except Exception as e:
        # Container teardown (podman stop/SIGTERM) can invalidate the shared
        # rclpy context out from under spin()'s wait-set before this node
        # gets a clean KeyboardInterrupt -- spin() then raises RCLError
        # ("the given context is not valid") instead. That's an expected
        # shutdown-race, not a real fault (all real decompression work is
        # already done by the time teardown starts) -- fall through to the
        # same teardown path below instead of letting the traceback surface.
        print(f"[decompress_rgbd] spin() ended by context shutdown ({e}); "
              f"shutting down normally.", flush=True)
    finally:
        print(f"[decompress_rgbd] decompress done: color={node.n_c} depth={node.n_d}", flush=True)
        # Issue #865: a plain `print` rather than the node logger -- by this
        # point the rclpy context may already be torn down, which would make a
        # logger call the one thing that swallows the final accounting.
        print(
            f"[decompress_rgbd] FINAL LEDGER color arrived={node.arrived_c} "
            f"published={node.n_c} decode_fail={node.decode_fail_c} "
            f"last_stamp={node.last_c_stamp:.6f} | depth arrived={node.arrived_d} "
            f"published={node.n_d} decode_fail={node.decode_fail_d} "
            f"last_stamp={node.last_d_stamp:.6f}",
            flush=True,
        )
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
