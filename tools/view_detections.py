#!/usr/bin/env python3
"""検出結果を画像に重ねて表示する。**PC 側で動かす**ことを想定。

Pi 側の負荷は「既に publish している画像を 1 つ多く購読される」分だけで、描画も
ウィンドウも PC が持つ。Pi では動かさないこと (CPU が飽和して認識周期が落ちる)。

    # [PC] Pi と DDS が通っている状態で
    python3 view_detections.py

    python3 view_detections.py --save out.mp4               # 表示しつつ録画
    python3 view_detections.py --save out.mp4 --no-window   # 録画だけ (表示しない)
    python3 view_detections.py --no-window                  # ヘッドレス。統計だけ出す

画像と検出は別トピックなので、**その時点で最後に届いた検出**を重ねる (時刻照合はしない)。
検出は画像より遅いので、同じ検出が数フレームにまたがって表示されるのは正常。
"""
from __future__ import annotations

import argparse

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image

from umiusi_autonomy_msgs.msg import BalloonDetectionArray

# BGR。検出器の色ラベルに合わせる
COLOURS = {"red": (0, 0, 255), "blue": (255, 128, 0), "yellow": (0, 200, 255)}


class Viewer(Node):
    def __init__(self, a):
        super().__init__("view_detections")
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Image, a.image_topic, self._on_image, qos)
        self.create_subscription(BalloonDetectionArray, a.det_topic, self._on_det, 10)
        self.bridge = CvBridge()
        self.args = a
        self.dets = None
        self.writer = None
        self.n_img = 0
        self.n_det = 0
        self.create_timer(5.0, self._stats)
        self.get_logger().info(f"画像 '{a.image_topic}' + 検出 '{a.det_topic}' を表示")

    def _on_det(self, msg):
        self.dets = msg
        self.n_det += 1

    def _stats(self):
        d = len(self.dets.detections) if self.dets else 0
        self.get_logger().info(f"画像 {self.n_img} 枚 / 検出 {self.n_det} 回 / 直近の検出数 {d}")

    def _on_image(self, msg):
        self.n_img += 1
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception as e:  # noqa: BLE001
            self.get_logger().warning(f"画像を変換できません: {e}", throttle_duration_sec=5.0)
            return
        img = self._draw(img.copy())

        if self.args.save:
            if self.writer is None:
                h, w = img.shape[:2]
                self.writer = cv2.VideoWriter(self.args.save,
                                              cv2.VideoWriter_fourcc(*"mp4v"),
                                              self.args.save_fps, (w, h))
            self.writer.write(img)
        if not self.args.no_window:
            cv2.imshow("umiusi detections", img)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                raise SystemExit(0)

    def _draw(self, img):
        h, w = img.shape[:2]
        if self.dets is None:
            cv2.putText(img, "no detections yet", (8, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
            return img
        for d in self.dets.detections:
            col = COLOURS.get(d.colour, (255, 255, 255))
            if len(d.bbox) == 4:
                x0, y0, x1, y1 = (int(v) for v in d.bbox)
                # 検出器の入力サイズと表示サイズが違う場合に備えて画面内に丸める
                x0, x1 = max(0, min(w - 1, x0)), max(0, min(w - 1, x1))
                y0, y1 = max(0, min(h - 1, y0)), max(0, min(h - 1, y1))
                cv2.rectangle(img, (x0, y0), (x1, y1), col, 2)
                label = f"{d.colour} {d.confidence:.2f} {d.range_m:.1f}m"
                cv2.putText(img, label, (x0, max(12, y0 - 4)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, col, 1, cv2.LINE_AA)
        cv2.putText(img, f"{len(self.dets.detections)} det", (8, 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        return img

    def destroy_node(self):
        if self.writer is not None:
            self.writer.release()
        if not self.args.no_window:
            cv2.destroyAllWindows()
        return super().destroy_node()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--image-topic", default="/front_cam/image_raw")
    ap.add_argument("--det-topic", default="/perception_node/detections")
    ap.add_argument("--save", default="", help="mp4 に保存する")
    ap.add_argument("--save-fps", type=float, default=15.0,
                    help="保存動画のフレームレート (実測の供給レートに合わせる)")
    ap.add_argument("--no-window", action="store_true", help="ウィンドウを出さない")
    a = ap.parse_args()

    rclpy.init()
    node = Viewer(a)
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
