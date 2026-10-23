#!/usr/bin/env python3
"""ROS 2 node: opens a V4L2 camera directly and publishes whether the bucket is empty.

Designed for a Raspberry Pi (CPU only): low capture resolution, low inference
rate, limited torch threads.
"""

from collections import deque

import cv2
import torch
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, Float32

from bucket_empty_detector.bucket_empty_detector import BucketDetector


class BucketEmptyNode(Node):
    def __init__(self):
        super().__init__("bucket_empty_detector")
        self.declare_parameter("camera_device", "/dev/video0")  # path or index
        self.declare_parameter("model_path", "empty_model.pt")
        self.declare_parameter("frame_width", 640)
        self.declare_parameter("frame_height", 480)
        self.declare_parameter("rate_hz", 2.0)
        self.declare_parameter(
            "roi", [0, 0, 0, 0]
        )  # x y w h in captured frame; 0 w/h = full
        self.declare_parameter("vote_window", 5)
        self.declare_parameter("threshold_override", 0.0)  # >0 replaces saved threshold
        self.declare_parameter("num_threads", 3)  # leave a core free on the Pi

        p = self.get_parameter
        torch.set_num_threads(int(p("num_threads").value))

        self.det = BucketDetector(device="cpu")
        self.det.load(p("model_path").value)
        if p("threshold_override").value > 0:
            self.det.threshold = float(p("threshold_override").value)

        self.roi = list(p("roi").value)
        self.votes = deque(maxlen=int(p("vote_window").value))
        self.cap = None
        self._open_camera()

        self.pub_empty = self.create_publisher(Bool, "bucket_empty", 10)
        self.pub_score = self.create_publisher(Float32, "bucket_anomaly_score", 10)
        self.create_timer(1.0 / float(p("rate_hz").value), self.tick)
        self.get_logger().info(
            f"camera={p('camera_device').value} threshold={self.det.threshold:.3f}"
        )

    def _open_camera(self):
        dev = self.get_parameter("camera_device").value
        if isinstance(dev, str) and dev.isdigit():
            dev = int(dev)
        cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        if not cap.isOpened():
            self.get_logger().error(f"cannot open camera {dev}")
            self.cap = None
            return
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(self.get_parameter("frame_width").value))
        cap.set(
            cv2.CAP_PROP_FRAME_HEIGHT, int(self.get_parameter("frame_height").value)
        )
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # always process the newest frame
        self.cap = cap

    def tick(self):
        if self.cap is None:
            self._open_camera()
            return
        # drop any stale buffered frames, then read the latest
        for _ in range(2):
            self.cap.grab()
        ok, bgr = self.cap.retrieve()
        if not ok or bgr is None:
            self.get_logger().warn("camera read failed, reopening")
            self.cap.release()
            self.cap = None
            return

        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        x, y, w, h = self.roi
        if w > 0 and h > 0:
            rgb = rgb[y : y + h, x : x + w]

        empty, score = self.det.is_empty(rgb)
        self.votes.append(empty)
        smoothed = sum(self.votes) > len(self.votes) / 2

        self.pub_empty.publish(Bool(data=bool(smoothed)))
        self.pub_score.publish(Float32(data=float(score)))

    def destroy_node(self):
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main():
    rclpy.init()
    node = BucketEmptyNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
