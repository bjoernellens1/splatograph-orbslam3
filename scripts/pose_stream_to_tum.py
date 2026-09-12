#!/usr/bin/env python3
"""Subscribe to a geometry_msgs/PoseStamped topic and append TUM lines
(timestamp tx ty tz qx qy qz qw) to a text file, flushing per message.

Replacement for `ros2 bag record` in the graceful-shutdown runner: rosbag2's
recorder hung on SIGINT in the k1final image and left a 0-byte mcap, losing the
whole causal /slam/pose stream. This writer loses nothing on any signal.

usage: pose_stream_to_tum.py <topic> <out.txt> [comment]
"""
import sys, signal
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped


class Writer(Node):
    def __init__(self, topic, path, comment):
        super().__init__("pose_stream_to_tum")
        self.f = open(path, "w", buffering=1)
        self.f.write(f"# timestamp tx ty tz qx qy qz qw ({comment}; raw ROS message frame)\n")
        self.n = 0
        self.create_subscription(PoseStamped, topic, self.cb, 100)

    def cb(self, m):
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        p, q = m.pose.position, m.pose.orientation
        self.f.write(f"{t:.9f} {p.x:.9f} {p.y:.9f} {p.z:.9f} {q.x:.9f} {q.y:.9f} {q.z:.9f} {q.w:.9f}\n")
        self.n += 1


def main():
    topic, path = sys.argv[1], sys.argv[2]
    comment = sys.argv[3] if len(sys.argv) > 3 else f"live {topic} stream"
    rclpy.init(signal_handler_options=getattr(__import__("rclpy.signals", fromlist=["SignalHandlerOptions"]), "SignalHandlerOptions").NO)
    node = Writer(topic, path, comment)
    stop = {"v": False}
    signal.signal(signal.SIGINT, lambda *_: stop.__setitem__("v", True))
    signal.signal(signal.SIGTERM, lambda *_: stop.__setitem__("v", True))
    while not stop["v"]:
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_node(); rclpy.shutdown()
    node.f.flush(); node.f.close()
    print(f"[pose_stream_to_tum] wrote {node.n} poses to {path}")


if __name__ == "__main__":
    main()
