"""perception_node — onboard balloon detection, a THIN rclpy wrapper around the shared library.

Subscribes the onboard camera (sensor_msgs/Image), runs the learned detector
(``umiusi_perception.learned_detector``) followed by the near-range red/blue colour
re-confirmation (``umiusi_perception.sanitise_near_colours``) — EXACTLY the perception half of
``tools/autonomy_run`` — and publishes the per-frame detections as ``BalloonDetectionArray``. All
detection logic lives in the ROS-free ``umiusi_perception`` package; this node only does topic plumbing and
message conversion, so the same detector runs bit-identically in sim and on the robot.

The heavy imports (torch, umiusi_perception) are deferred until the first image so ``colcon build`` and
``--help`` do not require them; on the Pi they are imported once at startup.

Parameters
----------
model_path      : learned detector checkpoint (.pt). 未指定なら同梱の camp_real2.pt。
image_topic     : onboard camera topic (default /front_cam/image_raw).
detections_topic: output topic (default ~/detections).
image_timeout   : この秒数だけ画像が来なければ警告する (default 5.0, 0 = 無効)。
conf_thresh     : detector confidence floor (default: the checkpoint's stored value).
input_size      : detector square input (default: the checkpoint's stored value).
fovy_deg        : camera vertical FOV, must match the physical camera (default 60.0).
max_rate_hz     : cap the detector rate; frames arriving faster are dropped (default 10 Hz, 0 = no cap).
                  位相追従で間引くので、入力が上限より速くても出力は上限に張り付く。
sanitise_near   : run the near red/blue colour re-confirmation (default True).
"""

from __future__ import annotations

from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import Image

from umiusi_autonomy_msgs.msg import BalloonDetection, BalloonDetectionArray

from umiusi_autonomy.image_convert import image_to_rgb
from umiusi_autonomy.rate_limiter import RateLimiter


class PerceptionNode(Node):
    def __init__(self):
        super().__init__("perception_node")
        self.declare_parameter("model_path", "")
        self.declare_parameter("image_topic", "/front_cam/image_raw")
        self.declare_parameter("detections_topic", "~/detections")
        self.declare_parameter("conf_thresh", -1.0)   # <0 -> use the checkpoint's stored floor
        self.declare_parameter("input_size", 0)        # 0 -> use the checkpoint's stored size
        self.declare_parameter("fovy_deg", 60.0)
        self.declare_parameter("max_rate_hz", 10.0)
        self.declare_parameter("sanitise_near", True)
        # 画像が来ていないことに気付けるようにする。**画像ゼロでも無言で回り続ける**ので、
        # 8/25 の水中 run では 15.6 分間ずっと検出ゼロ (= FSM が SEARCH から出られない) だった
        # ことに、bag を持ち帰るまで気付けなかった。0 以下で無効。
        self.declare_parameter("image_timeout", 5.0)

        self._model_path = str(self.get_parameter("model_path").value).strip()
        if not self._model_path:
            # 未指定なら同梱の検出器を使う (clone しただけで動くように)。既定は
            # **camp_real2.pt** — 8/25 のプール実写をハードネガティブとして継続学習した版で、
            # 実プールでの precision が 0.29 -> 0.78 (F1 0.44 -> 0.80)。旧 camp_real の
            # 「4.6 個/枚の誤検出」「右下の固定誤検出」はこれで解消した。
            # conf_thresh は checkpoint に 0.4 が入っているので指定不要。
            # 旧版を使いたいときは model_path で明示する。models/detector/README.md 参照。
            self._model_path = str(Path(get_package_share_directory("umiusi_autonomy"))
                                   / "models" / "detector" / "camp_real2.pt")
        self._fovy = float(self.get_parameter("fovy_deg").value)
        self._sanitise = bool(self.get_parameter("sanitise_near").value)
        # 位相追従の間引き。素朴に「通した時刻から一定時間空ける」方式だと、入力が上限より
        # わずかに速いだけで 1 フレームおきに落ち、目標の半分近くまで下がる
        # (実機: 15 Hz 入力 + 10 Hz 上限 -> 7.9 Hz)。RateLimiter を参照。
        self._limiter = RateLimiter(float(self.get_parameter("max_rate_hz").value))
        self._warned_no_stamp = False
        self._image_timeout = float(self.get_parameter("image_timeout").value)
        self._last_image_t = None      # None = まだ 1 枚も来ていない
        self._n_images = 0

        self._detector = None       # lazily loaded on the first frame (defer torch import)
        self._sanitise_fn = None

        image_topic = self.get_parameter("image_topic").value
        det_topic = self.get_parameter("detections_topic").value
        self._pub = self.create_publisher(BalloonDetectionArray, det_topic, 10)
        self._sub = self.create_subscription(Image, image_topic, self._on_image, 1)
        self._image_topic = image_topic
        if self._image_timeout > 0.0:
            self._watchdog = self.create_timer(self._image_timeout, self._check_image_flow)

        if not self._model_path:
            self.get_logger().error(
                "parameter 'model_path' is empty — set it to a learned detector .pt checkpoint")
        self.get_logger().info(
            f"perception_node: image='{image_topic}' -> detections='{det_topic}' "
            f"(fovy={self._fovy:.0f}deg, max_rate={self._limiter.rate_hz:.0f}Hz, sanitise_near={self._sanitise})")

    def _check_image_flow(self):
        """画像が途切れていないか (そもそも来ているか) を見張る。実機カメラは RTSP なので
        ``camera_bridge_node`` が居ないと 1 枚も来ない。沈黙で気付けないのが一番困る。"""
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._last_image_t is None:
            self.get_logger().warning(
                f"'{self._image_topic}' に画像が 1 枚も来ていません "
                "(camera_bridge_node は起動していますか? use_camera_bridge:=true)",
                throttle_duration_sec=10.0)
            return
        gap = now - self._last_image_t
        if gap > self._image_timeout:
            self.get_logger().warning(
                f"'{self._image_topic}' の画像が {gap:.1f} s 途切れています "
                f"({self._n_images} 枚受信済み)",
                throttle_duration_sec=10.0)

    def _ensure_detector(self):
        """Load the detector on first use (defers the torch/umiusi_perception import off the build path)."""
        if self._detector is not None:
            return True
        try:
            from umiusi_perception.learned_detector import load_learned_detector
            from umiusi_perception.tracker import sanitise_near_colours
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(
                f"cannot import the detector from umiusi_perception ({type(e).__name__}: {e}); "
                "is the umiusi_perception wheel installed (pip install .../packages/perception)?",
                throttle_duration_sec=10.0)
            return False
        conf = self.get_parameter("conf_thresh").value
        size = self.get_parameter("input_size").value
        try:
            self._detector = load_learned_detector(
                self._model_path,
                input_size=(int(size) if int(size) > 0 else None),
                conf_thresh=(float(conf) if float(conf) >= 0.0 else None),
                fovy_deg=self._fovy,
            )
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"failed to load detector '{self._model_path}': "
                                    f"{type(e).__name__}: {e}")
            return False
        self._sanitise_fn = sanitise_near_colours
        self.get_logger().info(f"detector loaded from '{self._model_path}'")
        return True

    def _on_image(self, msg: Image):
        # ウォッチドッグ用。**レート制限より前**に記録する — 落としたフレームも「来ている」ので。
        self._last_image_t = self.get_clock().now().nanoseconds * 1e-9
        self._n_images += 1
        # rate cap: drop frames that arrive faster than max_rate_hz (realistic Pi-4 detector timing).
        # ヘッダの stamp を使うが、**設定していない publisher だと 0 のまま進まず全フレームが
        # 落ちて perception が沈黙する**ので、その場合はノードの時計に切り替える。
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if stamp <= 0.0:
            if not self._warned_no_stamp:
                self._warned_no_stamp = True
                self.get_logger().warning(
                    "画像の header.stamp が設定されていません。レート制限にノードの時計を使います")
            stamp = self.get_clock().now().nanoseconds * 1e-9
        if not self._limiter.allow(stamp):
            return
        if not self._model_path or not self._ensure_detector():
            return
        try:
            rgb = image_to_rgb(msg)
        except ValueError as e:
            self.get_logger().warn(str(e), throttle_duration_sec=5.0)
            return
        dets = self._detector(rgb)
        if self._sanitise:
            dets = self._sanitise_fn(rgb, dets)
        self._pub.publish(self._to_msg(msg.header, dets))

    @staticmethod
    def _to_msg(header, dets) -> BalloonDetectionArray:
        out = BalloonDetectionArray()
        out.header = header
        for d in dets:
            m = BalloonDetection()
            m.colour = d.colour
            m.points = int(d.points)
            m.azimuth = float(d.bearing[0])
            m.elevation = float(d.bearing[1])
            m.range_m = float(d.range_m)
            m.confidence = float(d.confidence)
            m.bbox = [int(x) for x in d.bbox]
            m.centroid = [float(d.centroid[0]), float(d.centroid[1])]
            m.area_px = int(d.area_px)
            out.detections.append(m)
        return out


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
