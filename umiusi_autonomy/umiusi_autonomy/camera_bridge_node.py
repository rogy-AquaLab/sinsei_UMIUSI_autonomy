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
from sensor_msgs.msg import Image

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
        self.declare_parameter("max_rate_hz", 15.0)   # publish レート上限 (0 = 取れるだけ)
        self.declare_parameter("latency_ms", 100)     # rtspsrc のジッタバッファ
        self.declare_parameter("hw_decode", True)     # False -> software デコード (avdec_h264)
        self.declare_parameter("reconnect_sec", 3.0)  # 読めなくなったときの再接続間隔

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

        self._open()
        period = 1.0 / rate if rate > 0 else 0.001
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
            self.get_logger().warning(
                "ハードウェア経路 (v4l2h264dec/v4l2convert) を開けません; software に落とします "
                "(CPU 消費が 5 倍程度になります)")
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
            if self._fail >= 10:
                self.get_logger().warning("フレームが取れないので再接続します",
                                          throttle_duration_sec=10.0)
                self._fail = 0
                self._open()
            return
        self._fail = 0
        if frame.shape[1] != self._w or frame.shape[0] != self._h:
            frame = cv2.resize(frame, (self._w, self._h), interpolation=cv2.INTER_AREA)
        msg = self._bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self._frame_id
        self._pub.publish(msg)
        self._n += 1
        if self._n % 300 == 0:
            self.get_logger().info(f"{self._n} フレーム中継")

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
