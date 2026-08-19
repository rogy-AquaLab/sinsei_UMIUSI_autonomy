#!/usr/bin/env python3
"""PC(G10) から sim 画像を ROS 2 カメラトピックとして配信する。

実機カメラは GStreamer/RTSP 経由で ROS トピックではないため、perception の動作確認用に
PC 側から画像を流し込む。Pi の perception_node がこれを購読する。

  python3 sim_image_pub.py --dir /home/satoimo/mujoco_ws/ai/balloon/sim_eval/images --rate 10
"""
from __future__ import annotations

import argparse
import glob
import os

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image


class SimImagePub(Node):
    def __init__(self, args):
        super().__init__("sim_image_pub")
        # カメラ映像は best-effort が実運用に近い（取りこぼしより遅延を嫌う）
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT if args.best_effort else ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pub = self.create_publisher(Image, args.topic, qos)
        self.bridge = CvBridge()
        self.args = args

        pats = ("*.jpg", "*.jpeg", "*.png")
        self.files = sorted(f for p in pats for f in glob.glob(os.path.join(args.dir, p)))
        if not self.files:
            raise SystemExit(f"画像が見つかりません: {args.dir}")

        self.idx = 0
        self.sent = 0
        self.get_logger().info(
            f"{len(self.files)}枚 を '{args.topic}' へ {args.rate}Hz で配信 "
            f"(QoS={'BEST_EFFORT' if args.best_effort else 'RELIABLE'}, "
            f"resize={'native' if args.width == 0 else f'{args.width}x{args.height}'})"
        )
        self.create_timer(1.0 / args.rate, self.tick)

    def tick(self):
        path = self.files[self.idx]
        img = cv2.imread(path, cv2.IMREAD_COLOR)
        if img is None:
            self.get_logger().warn(f"読めません: {path}")
        else:
            if self.args.width > 0:
                img = cv2.resize(img, (self.args.width, self.args.height), interpolation=cv2.INTER_AREA)
            msg = self.bridge.cv2_to_imgmsg(img, encoding="bgr8")
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = self.args.frame_id
            self.pub.publish(msg)
            self.sent += 1
            if self.sent % 50 == 0:
                self.get_logger().info(f"{self.sent} 枚配信 (現在: {os.path.basename(path)})")

        self.idx += 1
        if self.idx >= len(self.files):
            if self.args.loop:
                self.idx = 0
            else:
                self.get_logger().info(f"全 {len(self.files)} 枚を配信し終えました")
                raise SystemExit(0)


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--dir", default="/home/satoimo/mujoco_ws/ai/balloon/sim_eval/images")
    p.add_argument("--topic", default="/front_cam/image_raw")
    p.add_argument("--rate", type=float, default=10.0)
    p.add_argument("--width", type=int, default=0, help="0=元サイズ")
    p.add_argument("--height", type=int, default=0)
    p.add_argument("--frame-id", default="front_cam_optical")
    p.add_argument("--loop", action="store_true", default=True)
    p.add_argument("--best-effort", action="store_true", default=False,
                   help="既定は RELIABLE (perception_node の購読 QoS に一致)")
    args = p.parse_args()

    rclpy.init()
    node = SimImagePub(args)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
