"""perception_node — onboard balloon detection, a THIN rclpy wrapper around the shared library.

Subscribes the onboard camera (sensor_msgs/Image), runs the learned detector
(umiusi_perception.learned_detector) followed by the near-range red/blue colour
re-confirmation (umiusi_perception.sanitise_near_colours) — EXACTLY the perception half of
tools/autonomy_run — and publishes the per-frame detections as BalloonDetectionArray. All
detection logic lives in the ROS-free umiusi_perception package; this node only does topic plumbing and
message conversion, so the same detector runs bit-identically in sim and on the robot.

The heavy imports (torch, umiusi_perception) are deferred until the first image so colcon build and
--help do not require them; on the Pi they are imported once at startup.

パラメータは declare_parameter を参照。fovy_deg だけは実カメラと一致していること
(方位角の換算に直接効く — known_issues A-14)。
"""

from __future__ import annotations

from pathlib import Path

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import Image

from sinsei_umiusi_msgs.msg import RobotState
from umiusi_autonomy_msgs.msg import BalloonDetection, BalloonDetectionArray

from umiusi_autonomy.image_convert import image_to_rgb
from umiusi_autonomy.rate_limiter import RateLimiter



COLOURS = ("red", "yellow", "blue")


def filter_by_confidence(dets, floor: float, floor_by_colour: dict) -> list:
    """信頼度の足切り。色ごとの値が正ならそれ、そうでなければ floor。どちらも 0 以下なら落とさない。"""
    def keep(d):
        f = floor_by_colour.get(d.colour, -1.0)
        f = f if f > 0.0 else floor
        return f <= 0.0 or float(d.confidence) >= f
    return [d for d in dets if keep(d)]

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
        # **後段の信頼度フィルタ。** 検出器そのものの閾値 (conf_thresh) は重みを読んだ時点で
        # 固定されるので、実行中には動かせない。プールサイドで誤検出を絞りたいときのために、
        # publish の直前でもう一度足切りする。0 以下で無効 (検出器の閾値のまま)。
        # **上げる方向にしか効かない** — 検出器が出さなかったものは戻せない。
        # 8/25 のプール run では camp_real @0.30 が 4.6 個/枚の誤検出を出し、
        # ハードネガティブ再学習 (camp_real2 @0.40) で 267 -> 3 に落ちている。
        # 2026-10-01 のプール映像で camp_real2 は赤い風船を 0/36 (yellow と取り違える)。2026-10-03 から既定は
        # balloon_F320、2026-10-08 から balloon_F320_20261007。運用値は config/autonomy.yaml (models/detector/README.md)
        self.declare_parameter("min_confidence", -1.0)
        # 色ごとの足切り。0 以下ならその色は min_confidence に従う。赤だけ上げる用途
        # (風船の下の重りを red と誤検出し、FSM は 3 フレーム続くと突進する。2026-10-03 の映像)
        for colour in COLOURS:
            self.declare_parameter(f"min_confidence_{colour}", -1.0)
        # 断の検出用。画像ゼロでも無言で回り続ける (known_issues A-18)。0 以下で無効
        self.declare_parameter("image_timeout", 5.0)
        # AUTO 以外では推論しない。認識を使うのは AUTO の auto_target_generator だけなのに、
        # MANUAL / STANDBY でも 1 コアを推論で埋めていた (2026-10-03 実機)。
        # `/robot_state` が 1 度も来ない (core なし: mode:=navigator や use_core:=false) 間は止めない。
        # MANUAL の試験でも検出を bag に残したいときは false (`ros2 param set` で変えられる)
        self.declare_parameter("infer_only_in_auto", True)
        self.declare_parameter("robot_state_topic", "/robot_state")

        self._model_path = str(self.get_parameter("model_path").value).strip()
        if not self._model_path:
            # 未指定なら同梱の検出器。版の比較と切り替えは models/detector/README.md
            self._model_path = str(Path(get_package_share_directory("umiusi_autonomy"))
                                   / "models" / "detector" / "balloon_F320_20261007.pt")
        self._fovy = float(self.get_parameter("fovy_deg").value)
        self._sanitise = bool(self.get_parameter("sanitise_near").value)
        self._min_conf = float(self.get_parameter("min_confidence").value)
        self._min_conf_by_colour = {
            c: float(self.get_parameter(f"min_confidence_{c}").value) for c in COLOURS}
        self.add_on_set_parameters_callback(self._on_params)
        # 位相追従の間引き。素朴な「一定時間空ける」方式は入力がわずかに速いだけで
        # 1 フレームおきに落ち、目標の半分近くまで下がる。RateLimiter 参照
        self._limiter = RateLimiter(float(self.get_parameter("max_rate_hz").value))
        self._warned_no_stamp = False
        self._image_timeout = float(self.get_parameter("image_timeout").value)
        self._last_image_t = None      # None = まだ 1 枚も来ていない
        self._n_images = 0
        self._only_in_auto = bool(self.get_parameter("infer_only_in_auto").value)
        self._robot_state = None       # None = /robot_state がまだ来ていない (core なし)
        self._preload_tried = False

        self._detector = None       # lazily loaded on the first frame (defer torch import)
        self._sanitise_fn = None

        image_topic = self.get_parameter("image_topic").value
        det_topic = self.get_parameter("detections_topic").value
        self._pub = self.create_publisher(BalloonDetectionArray, det_topic, 10)
        self._sub = self.create_subscription(Image, image_topic, self._on_image, 1)
        self._image_topic = image_topic
        self._sub_state = self.create_subscription(
            RobotState, self.get_parameter("robot_state_topic").value, self._on_robot_state, 10)
        if self._image_timeout > 0.0:
            self._watchdog = self.create_timer(self._image_timeout, self._check_image_flow)

        if not self._model_path:
            self.get_logger().error(
                "parameter 'model_path' is empty — set it to a learned detector .pt checkpoint")
        self.get_logger().info(
            f"perception_node: image='{image_topic}' -> detections='{det_topic}' "
            f"(fovy={self._fovy:.0f}deg, max_rate={self._limiter.rate_hz:.0f}Hz, sanitise_near={self._sanitise}, "
            f"infer_only_in_auto={self._only_in_auto})")

    def _on_robot_state(self, msg: RobotState):
        prev = self._robot_state
        self._robot_state = int(msg.state)
        if self._only_in_auto and (prev == RobotState.AUTO) != (self._robot_state == RobotState.AUTO):
            if self._robot_state == RobotState.AUTO:
                self.get_logger().info("AUTO になったので認識を始めます")
            else:
                self.get_logger().info(f"AUTO ではない (state={self._robot_state}) ので認識を止めます")

    def _inference_enabled(self) -> bool:
        if not self._only_in_auto or self._robot_state is None:
            return True
        return self._robot_state == RobotState.AUTO

    def _check_image_flow(self):
        """画像が途切れていないか (そもそも来ているか) を見張る。実機カメラは RTSP なので
        camera_bridge_node が居ないと 1 枚も来ない。沈黙で気付けないのが一番困る。"""
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

    def _on_params(self, params):
        """`ros2 param set /perception_node min_confidence 0.5` を実行中に効かせる。

        誤検出の絞り込みは**現場で回しながら**でないと当たりが分からない。再起動すると
        検出器の読み込み (数秒) と映像の再購読が挟まるので、走らせたまま変えたい。
        """
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            if p.name == "min_confidence":
                try:
                    self._min_conf = float(p.value)
                except (TypeError, ValueError) as e:
                    return SetParametersResult(successful=False, reason=str(e))
                self.get_logger().warning(
                    f"min_confidence={self._min_conf:.2f}"
                    f"{' (無効)' if self._min_conf <= 0.0 else ''}")
            elif p.name.startswith("min_confidence_") and p.name[len("min_confidence_"):] in COLOURS:
                try:
                    self._min_conf_by_colour[p.name[len("min_confidence_"):]] = float(p.value)
                except (TypeError, ValueError) as e:
                    return SetParametersResult(successful=False, reason=str(e))
                self.get_logger().warning(f"{p.name}={float(p.value):.2f}")
            # 濁りの程度で当たりが変わる 2 つ。**検出器を読み直さずに変えられる**ので
            # ここで受ける (`conf_thresh` は重みを読んだ時点で焼き込まれるので変えられない)
            elif p.name == "sanitise_near":
                self._sanitise = bool(p.value)
                self.get_logger().warning(f"sanitise_near={self._sanitise}")
            elif p.name == "max_rate_hz":
                try:
                    self._limiter = RateLimiter(float(p.value))
                except (TypeError, ValueError) as e:
                    return SetParametersResult(successful=False, reason=str(e))
                self.get_logger().warning(f"max_rate_hz={self._limiter.rate_hz:.1f}")
            elif p.name == "infer_only_in_auto":
                self._only_in_auto = bool(p.value)
                self.get_logger().warning(f"infer_only_in_auto={self._only_in_auto}")
        return SetParametersResult(successful=True)

    def _on_image(self, msg: Image):
        # ウォッチドッグ用。レート制限より前に記録する — 落としたフレームも「来ている」ので。
        self._last_image_t = self.get_clock().now().nanoseconds * 1e-9
        self._n_images += 1
        if not self._inference_enabled():
            # 検出器の読み込み (数秒) だけは先に済ませる。AUTO に入った瞬間に止まらないように。
            # 失敗したら AUTO に入るまで再試行しない (毎フレーム読み直して CPU を食わないように)
            if self._model_path and not self._preload_tried:
                self._preload_tried = True
                if self._ensure_detector():
                    # 起動の段の待ち (bringup の wait_perception / umiusi_stack.sh の wait_topic) は
                    # 「検出器の読み込み + 初フレーム」を最初の detections で見ている。空を 1 回だけ出す
                    self._pub.publish(self._to_msg(msg.header, []))
            return
        # ヘッダの stamp を使うが、設定していない publisher だと 0 のまま進まず全フレームを
        # 落として沈黙するので、その場合はノードの時計に切り替える
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
        n_before = len(dets)
        dets = filter_by_confidence(dets, self._min_conf, self._min_conf_by_colour)
        if n_before != len(dets):
            self.get_logger().info(
                f"信頼度の足切り (min_confidence={self._min_conf:.2f}、色ごと {self._min_conf_by_colour}) で "
                f"{n_before - len(dets)} 件を落とした", throttle_duration_sec=5.0)
        self._pub.publish(self._to_msg(msg.header, dets))
        # 末尾でも更新する。初回は _ensure_detector() の同期ロードが image_timeout を
        # 超えることがあり、そのままだと復帰直後に偽の「画像が途切れた」警告が出る
        self._last_image_t = self.get_clock().now().nanoseconds * 1e-9

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
