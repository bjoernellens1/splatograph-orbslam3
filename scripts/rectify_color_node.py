#!/usr/bin/env python3
"""Explicit image rectification for recorded Orbbec bag development replays.

This node is intentionally *not* part of the live-camera path: a live Orbbec
must provide hardware-driver undistortion before it reaches the RGB-D sync
owner.  Some legacy recordings contain distorted compressed colour frames,
however.  Those frames cannot truthfully cross Splatograph's pinhole ingress
contract until their pixels are remapped and their CameraInfo is changed to the
matching zero-distortion pinhole model.

The node preserves every image header exactly and never pairs colour/depth:
splatograph-rgbd-sync remains the sole RGB-D pairing owner downstream.  Use
``interpolation:=nearest`` for metric depth, so remapping does not fabricate
interpolated depth values.
"""

from __future__ import annotations

import copy

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


def rectified_camera_info(info: CameraInfo) -> CameraInfo:
    """Return the pinhole CameraInfo matching an undistorted image."""

    if info.width <= 0 or info.height <= 0 or len(info.k) != 9:
        raise ValueError("CameraInfo must contain positive dimensions and a 3x3 K")
    result = copy.deepcopy(info)
    result.d = [0.0] * len(info.d)
    result.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
    result.p = [
        float(info.k[0]), 0.0, float(info.k[2]), 0.0,
        0.0, float(info.k[4]), float(info.k[5]), 0.0,
        0.0, 0.0, 1.0, 0.0,
    ]
    return result


class RectifyImage(Node):
    """Publish actual remapped pixels and their matching CameraInfo."""

    def __init__(self) -> None:
        super().__init__("rectify_image")
        string = lambda name, default: self.declare_parameter(name, default).value
        self.image_in = string("image_in", "/camera/color/image_raw")
        self.info_in = string("camera_info_in", "/camera/color/camera_info")
        self.image_out = string("image_out", "/camera/color/image_rect")
        self.info_out = string("camera_info_out", "/camera/color/camera_info_rect")
        interpolation = string("interpolation", "linear")
        interpolation_modes = {
            "linear": cv2.INTER_LINEAR,
            "nearest": cv2.INTER_NEAREST,
        }
        if interpolation not in interpolation_modes:
            raise ValueError("interpolation must be 'linear' or 'nearest'")
        self._interpolation = interpolation_modes[interpolation]
        self.bridge = CvBridge()
        self._info: CameraInfo | None = None
        self._rectified_info: CameraInfo | None = None
        self._map_x: np.ndarray | None = None
        self._map_y: np.ndarray | None = None
        self._published = 0
        self.pub_image = self.create_publisher(Image, self.image_out, 30)
        self.pub_info = self.create_publisher(CameraInfo, self.info_out, 30)
        self.create_subscription(CameraInfo, self.info_in, self._on_info, qos_profile_sensor_data)
        self.create_subscription(Image, self.image_in, self._on_image, qos_profile_sensor_data)
        self.get_logger().info(
            f"explicit bag-only rectifier ({interpolation}): {self.image_in}->{self.image_out}; "
            f"{self.info_in}->{self.info_out}"
        )

    def _on_info(self, info: CameraInfo) -> None:
        k = np.asarray(info.k, dtype=np.float64).reshape(3, 3)
        d = np.asarray(info.d, dtype=np.float64)
        if info.width <= 0 or info.height <= 0 or not np.isfinite(k).all() or k[0, 0] <= 0 or k[1, 1] <= 0:
            self.get_logger().error("refusing invalid CameraInfo for rectification")
            return
        self._info = copy.deepcopy(info)
        self._rectified_info = rectified_camera_info(info)
        self._map_x, self._map_y = cv2.initUndistortRectifyMap(
            k, d, None, k, (int(info.width), int(info.height)), cv2.CV_32FC1
        )

    def _on_image(self, msg: Image) -> None:
        if self._map_x is None or self._map_y is None or self._rectified_info is None:
            return
        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        if image.shape[1] != self._map_x.shape[1] or image.shape[0] != self._map_x.shape[0]:
            self.get_logger().error("image dimensions do not match CameraInfo; refusing frame")
            return
        rectified = cv2.remap(image, self._map_x, self._map_y, self._interpolation)
        output = self.bridge.cv2_to_imgmsg(rectified, encoding=msg.encoding)
        output.header = msg.header
        info = copy.deepcopy(self._rectified_info)
        info.header = msg.header
        self.pub_image.publish(output)
        self.pub_info.publish(info)
        self._published += 1


def main() -> None:
    rclpy.init()
    node = RectifyImage()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f"rectify_image done: published={node._published}")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
