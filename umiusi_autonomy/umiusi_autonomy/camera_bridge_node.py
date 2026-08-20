"""camera_bridge_node — 実機カメラの RTSP ストリームを ROS の Image トピックへ橋渡しする。

``sinsei_umiusi_control`` の ``gst_camera_node`` は GStreamer パイプラインを起動するだけで、
画像を ROS トピックに publish しない (パラメータは ``pipeline`` 文字列のみ)。一方 perception_node は
``sensor_msgs/Image`` を購読するため、そのままでは実機カメラの映像が perception に届かない。
このノードがその欠落を埋める。control 側には一切手を入れない。

既定では GStreamer の **ハードウェア H.264 デコーダ** (``v4l2h264dec``) を使う。Raspberry Pi の
CPU で 720p を software デコードすると perception の取り分を食い潰すため、ここは重要。
``videoscale`` で publish 前に縮小するのも同じ理由 (640x480 の生 Image は 921 kB/frame あり、
RELIABLE QoS では転送だけで頭打ちになる)。

    ros2 run umiusi_autonomy camera_bridge_node --ros-args \
        -p rtsp_url:=rtsp://localhost:8554/cam1 -p width:=320 -p height:=240

QoS は RELIABLE 固定。perception_node の購読が RELIABLE のため、BEST_EFFORT にすると
``No messages will be received`` となって一切届かない。
"""

from __future__ import annotations

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image

# rtspsrc -> depay -> HW decode -> BGR へ変換して appsink。drop=true/max-buffers=1 で
# 遅いコンシューマに引きずられず、常に最新フレームだけを渡す。
# デコードとスケール/色変換の両方をハードウェアに逃がす。実測で software の
# videoconvert+videoscale が 102% CPU だったのに対し、v4l2convert は 19.9% で済む。
_HW_PIPELINE = (
    "rtspsrc location={url} latency={latency} protocols=tcp ! "
    "rtph264depay ! h264parse ! v4l2h264dec ! v4l2convert ! "
    "video/x-raw,format=BGR,width={w},height={h} ! "
    "appsink drop=true max-buffers=1 sync=false"
)

# v4l2convert が使えない環境向けのフォールバック (CPU を大きく食うので最後の手段)
_SW_PIPELINE = (
    "rtspsrc location={url} latency={latency} protocols=tcp ! "
    "rtph264depay ! h264parse ! {decoder} ! "
    "videoconvert ! videoscale ! "
    "video/x-raw,format=BGR,width={w},height={h} ! "
    "appsink drop=true max-buffers=1 sync=false"
)


class CameraBridge(Node):
    def __init__(self):
        super().__init__("camera_bridge_node")
        self.declare_parameter("rtsp_url", "rtsp://localhost:8554/cam1")
        self.declare_parameter("image_topic", "/front_cam/image_raw")
        self.declare_parameter("width", 320)          # publish する幅 (autonomy.yaml の frame_w と揃える)
        self.declare_parameter("height", 240)         # publish する高さ (frame_h と揃える)
        self.declare_parameter("frame_id", "front_cam_optical")
        # 既定は 0 = 制限なし。実測では制限をかけるとフレームを取りこぼし、
        # かえって perception のスループットが落ちた:
        #   制限なし  -> 供給 13.96 Hz / 認識 6.08 Hz
        #   12 Hz 制限 -> 供給  5.05 Hz / 認識 5.03 Hz
        # 供給を絞りたい場合はカメラ側の framerate (cameras.yaml) を下げるのが確実。
        self.declare_parameter("max_rate_hz", 0.0)
        self.declare_parameter("latency_ms", 100)     # rtspsrc のジッタバッファ
        self.declare_parameter("hw_decode", True)     # False -> software デコード (avdec_h264)
        self.declare_parameter("reconnect_sec", 3.0)  # 読めなくなったときの再接続間隔
        # --- ロギング: rosbag に残せる圧縮画像も出す (生 Image は 320x240 でも 3.5 MB/s ある) ---
        self.declare_parameter("publish_compressed", False)
        self.declare_parameter("jpeg_quality", 80)
        # --- 自動追従: consumer_topic の実レートに publish レートを合わせる ---
        # 注意: これが減らせるのは publish/シリアライズ/DDS の分だけで、デコード負荷は
        # カメラ側の fps で決まる (cameras.yaml の framerate)。デコードごと減らしたい場合は
        # カメラの framerate を目標認識周期の 1.5-2 倍程度に設定すること。
        self.declare_parameter("auto_rate", False)
        self.declare_parameter("consumer_topic", "/perception_node/detections")
        self.declare_parameter("auto_rate_margin", 1.2)   # 消費レートの何倍を供給するか
        self.declare_parameter("auto_rate_min", 2.0)
        self.declare_parameter("auto_rate_max", 30.0)
        self.declare_parameter("auto_rate_step", 1.0)   # 加算増加の刻み [Hz]

        self._url = str(self.get_parameter("rtsp_url").value)
        self._w = int(self.get_parameter("width").value)
        self._h = int(self.get_parameter("height").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._reconnect = float(self.get_parameter("reconnect_sec").value)
        rate = float(self.get_parameter("max_rate_hz").value)

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self._pub = self.create_publisher(Image, str(self.get_parameter("image_topic").value), qos)
        self._bridge = CvBridge()
        self._cap = None
        self._fail = 0
        self._n = 0
        self._last_reconnect = float("-inf")   # 初回は待たずに再接続する (sim time で 0 始まりでも)

        self._pub_c = None
        if bool(self.get_parameter("publish_compressed").value):
            self._pub_c = self.create_publisher(
                CompressedImage, str(self.get_parameter("image_topic").value) + "/compressed", qos)
            self._jpeg_q = int(self.get_parameter("jpeg_quality").value)
            self.get_logger().info(
                f"圧縮画像も publish します (JPEG q={self._jpeg_q}) — rosbag 用")

        self._auto = bool(self.get_parameter("auto_rate").value)
        self._target_dt = None      # auto_rate 時の目標間隔 [s]。None = 無制限で開始
        self._last_pub = 0.0
        if self._auto:
            self._consumer_stamps = []
            ctopic = str(self.get_parameter("consumer_topic").value)
            # 型を問わず到着だけ数えたいので、遅延バインドで購読する
            self._consumer_sub = None
            self._ctopic = ctopic
            self.create_timer(2.0, self._retune)
            self.get_logger().info(f"auto_rate: '{ctopic}' の実レートに追従します")

        self._open()
        # タイマ周期をそのまま目標レートにすると、カメラのフレーム到着周期とビートして
        # 取りこぼす (15 fps のカメラに 10 Hz タイマ -> 実測 6.8 Hz)。一方 1 ms まで速めると
        # read() を叩きすぎて逆にスループットが落ちる (実測 4.3 Hz)。
        # そこで「取得は目標の 2 倍で回し、publish は時間ゲートで間引く」形にする。
        self._fixed_dt = (1.0 / rate) if rate > 0 else None
        period = 1.0 / (rate * 2.0) if rate > 0 else 0.001
        self.create_timer(period, self._tick)

    # ------------------------------------------------------------------ capture
    def _pipeline(self, hw: bool) -> str:
        tpl = _HW_PIPELINE if hw else _SW_PIPELINE
        return tpl.format(
            url=self._url,
            latency=int(self.get_parameter("latency_ms").value),
            decoder="avdec_h264",
            w=self._w, h=self._h,
        )

    def _open(self) -> None:
        if self._cap is not None:
            self._cap.release()
            self._cap = None
        want_hw = bool(self.get_parameter("hw_decode").value)
        cap = cv2.VideoCapture(self._pipeline(want_hw), cv2.CAP_GSTREAMER)
        if not cap.isOpened() and want_hw:
            # RTSP が落ちている間は _reconnect 秒ごとにここを通るので、throttle しないと
            # 本当のエラーがログから流れてしまう (他の 2 つと同じ 10 秒に揃える)
            self.get_logger().warning(
                "ハードウェア経路 (v4l2h264dec/v4l2convert) を開けません; software に落とします "
                "(CPU 消費が 5 倍程度になります)", throttle_duration_sec=10.0)
            cap = cv2.VideoCapture(self._pipeline(False), cv2.CAP_GSTREAMER)
        if not cap.isOpened():
            self.get_logger().warning(
                f"GStreamer パイプラインを開けません; FFMPEG で {self._url} に直接接続します "
                "(software デコードになり CPU を食う点に注意)", throttle_duration_sec=10.0)
            cap = cv2.VideoCapture(self._url, cv2.CAP_FFMPEG)
        if cap.isOpened():
            self._cap = cap
            self.get_logger().info(f"接続しました: {self._url} -> {self._w}x{self._h}")
        else:
            self.get_logger().error(
                f"接続できません: {self._url} (RTSP サーバとカメラは動いていますか?)",
                throttle_duration_sec=10.0)

    # --------------------------------------------------------------------- loop
    def _tick(self) -> None:
        if self._cap is None:
            self._open()
            return
        ok, frame = self._cap.read()
        if not ok or frame is None:
            self._fail += 1
            # タイマは最速 (1 ms) で回りうるので、失敗回数だけを条件にすると
            # RTSP 断のあいだ 1 ms ごとに再接続を叩いてしまう。時間でも間隔を空ける。
            now = self.get_clock().now().nanoseconds * 1e-9
            if self._fail >= 10 and (now - self._last_reconnect) >= self._reconnect:
                self.get_logger().warning(
                    f"フレームが取れないので再接続します ({self._reconnect:.1f}秒間隔)",
                    throttle_duration_sec=10.0)
                self._fail = 0
                self._last_reconnect = now
                self._open()
            return
        self._fail = 0
        gate = self._target_dt if (self._auto and self._target_dt is not None) else self._fixed_dt
        if gate is not None:
            now = self.get_clock().now().nanoseconds * 1e-9
            if now - self._last_pub < gate * 0.98:   # 端数で 1 フレーム落とさないよう少し緩める
                return          # 供給過多なので publish を間引く (デコードは既に済んでいる)
            self._last_pub = now
        if frame.shape[1] != self._w or frame.shape[0] != self._h:
            frame = cv2.resize(frame, (self._w, self._h), interpolation=cv2.INTER_AREA)
        msg = self._bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        self._pub.publish(msg)
        if self._pub_c is not None:
            ok_enc, buf = cv2.imencode(".jpg", frame,
                                       [int(cv2.IMWRITE_JPEG_QUALITY), self._jpeg_q])
            if ok_enc:
                cm = CompressedImage()
                cm.header = msg.header
                cm.format = "jpeg"
                cm.data = buf.tobytes()
                self._pub_c.publish(cm)
        self._n += 1
        if self._n % 300 == 0:
            self.get_logger().info(f"{self._n} フレーム中継")

    def _retune(self) -> None:
        """consumer_topic の実レートを測り、publish レートをそれに合わせる。

        ここで例外を出しても画像の中継は止めない (ロギング/perception より優先度が低いため)。
        """
        try:
            self._retune_impl()
        except Exception as e:                           # noqa: BLE001
            self.get_logger().warning(f"auto_rate の調整に失敗: {type(e).__name__}: {e}",
                                      throttle_duration_sec=30.0)

    def _retune_impl(self) -> None:
        """AIMD で「消費側が捌ける最大レート」を探る。

        供給を単純に消費レートへ合わせると、起動直後などで消費が一時的に落ちたときに
        供給も落ち、消費がそれ以上出せなくなって二度と戻らない (デススパイラル)。
        そこで TCP と同じ考え方にする:

          * 消費が供給に追いついている  -> 供給を +step Hz して上限を探る (加算増加)
          * 消費が供給に追いつけていない -> 消費実測 x margin まで落とす (乗算減少)

        これで「消費側が本当に捌ける値」に収束し、負荷が軽くなれば自力で上がる。
        """
        if self._consumer_sub is None:
            names = dict(self.get_topic_names_and_types())
            if self._ctopic not in names:
                return
            try:
                from rosidl_runtime_py.utilities import get_message
                msg_type = get_message(names[self._ctopic][0])
            except Exception as e:                       # noqa: BLE001
                self.get_logger().warning(
                    f"auto_rate: '{self._ctopic}' の型を解決できません ({type(e).__name__}); "
                    "固定レートで動作します", throttle_duration_sec=30.0)
                return
            self._consumer_sub = self.create_subscription(
                msg_type, self._ctopic, self._on_consumer, 10)
            self.get_logger().info(f"auto_rate: '{self._ctopic}' を購読しました")
            return

        now = self.get_clock().now().nanoseconds * 1e-9
        self._consumer_stamps = [t for t in self._consumer_stamps if now - t < 6.0]
        lo = float(self.get_parameter("auto_rate_min").value)
        hi = float(self.get_parameter("auto_rate_max").value)
        step = float(self.get_parameter("auto_rate_step").value)
        margin = float(self.get_parameter("auto_rate_margin").value)

        supply = 1.0 / self._target_dt if self._target_dt else hi
        if len(self._consumer_stamps) < 3:
            # 消費が観測できない = 立ち上がり中。絞らずに待つ (ここで絞るとスパイラルになる)
            return
        span = self._consumer_stamps[-1] - self._consumer_stamps[0]
        if span <= 0:
            return
        consume = (len(self._consumer_stamps) - 1) / span

        if consume >= supply * 0.85:
            target = min(hi, supply + step)          # 追いついている -> 上を試す
            why = "追従できているので増やす"
        else:
            target = max(lo, consume * margin)       # 遅れている -> 消費実測まで落とす
            why = "消費が追いつかないので下げる"

        new_dt = 1.0 / target
        if self._target_dt is None or abs(new_dt - self._target_dt) / new_dt > 0.05:
            self._target_dt = new_dt
            self.get_logger().info(
                f"auto_rate: 消費 {consume:.2f} Hz / 供給 {supply:.2f} Hz -> "
                f"{target:.2f} Hz ({why})")

    def _on_consumer(self, _msg) -> None:
        self._consumer_stamps.append(self.get_clock().now().nanoseconds * 1e-9)

    def destroy_node(self):
        if self._cap is not None:
            self._cap.release()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraBridge()
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
