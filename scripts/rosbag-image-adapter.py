#!/usr/bin/env python3
import os
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64, String


class Orbslam3ImageAdapter(Node):
    def __init__(self) -> None:
        super().__init__("orbslam3_image_adapter")
        self.image_topic = os.environ.get("IMAGE_TOPIC", "/camera/color/image_raw")
        self.config_name = os.environ.get("CAMERA_CONFIG_NAME", "RealSense_D435i")
        self.send_config = True

        self.config_pub = self.create_publisher(String, "/mono_py_driver/experiment_settings", 1)
        self.image_pub = self.create_publisher(Image, "/mono_py_driver/img_msg", 1)
        self.time_pub = self.create_publisher(Float64, "/mono_py_driver/timestep_msg", 1)
        self.ack_sub = self.create_subscription(
            String, "/mono_py_driver/exp_settings_ack", self._ack_callback, 10
        )
        self.image_sub = self.create_subscription(Image, self.image_topic, self._image_callback, 10)
        self.config_timer = self.create_timer(0.05, self._publish_config)
        self.get_logger().info(
            f"bridging {self.image_topic} to /mono_py_driver/img_msg with config {self.config_name}"
        )

    def _publish_config(self) -> None:
        if not self.send_config:
            return
        msg = String()
        msg.data = self.config_name
        self.config_pub.publish(msg)

    def _ack_callback(self, msg: String) -> None:
        if msg.data == "ACK" and self.send_config:
            self.send_config = False
            self.get_logger().info("received ORB-SLAM3 config ACK")

    def _image_callback(self, msg: Image) -> None:
        if self.send_config:
            return
        timestamp = Float64()
        timestamp.data = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        self.time_pub.publish(timestamp)
        self.image_pub.publish(msg)


def main() -> None:
    rclpy.init()
    node = Orbslam3ImageAdapter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
