"""camera_bridge_node — 実機カメラの RTSP ストリームを ROS の Image トピックへ橋渡しする。

sinsei_umiusi_control の gst_camera_node は GStreamer パイプラインを起動するだけで、
画像を ROS トピックに publish しない (パラメータは pipeline 文字列のみ)。一方 perception_node は
sensor_msgs/Image を購読するため、そのままでは実機カメラの映像が perception に届かない。
このノードがその欠落を埋める。control 側には一切手を入れない。

既定では GStreamer の ハードウェア H.264 デコーダ (v4l2h264dec) を使う。Raspberry Pi の
CPU で 720p を software デコードすると perception の取り分を食い潰すため、ここは重要。
videoscale で publish 前に縮小するのも同じ理由 (640x480 の生 Image は 921 kB/frame あり、
RELIABLE QoS では転送だけで頭打ちになる)。

    ros2 run umiusi_autonomy camera_bridge_node --ros-args \
        -p rtsp_url:=rtsp://localhost:8554/cam1 -p width:=320 -p height:=240

shm_socket を渡すと RTSP ではなく共有メモリ (カメラ側 tee の分岐) から読み、デコードしない
(launch 引数 camera_source:=shm。docs/performance_tuning.md「共有メモリ経路」)。

QoS は RELIABLE 固定。perception_node の購読が RELIABLE のため、BEST_EFFORT にすると
No messages will be received となって一切届かない。
"""

from __future__ import annotations

import contextlib
import glob
import os
import threading
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CompressedImage, Image

# drop=true/max-buffers=1 で遅いコンシューマに引きずられず常に最新フレームを渡す。
# スケール/色変換も HW へ逃がすこと — software の videoconvert は CPU を 5 倍食う
# {rate} はデコード直後の間引き (max_fps)。以降の変換・Python・publish・perception の受信が全部減る
_HW_PIPELINE = (
    "rtspsrc location={url} latency={latency} protocols=tcp ! "
    "rtph264depay ! h264parse ! v4l2h264dec ! {rate}v4l2convert ! "
    "video/x-raw,format=BGR,width={w},height={h} ! "
    "appsink drop=true max-buffers=1 sync=false"
)

# v4l2convert が使えない環境向けのフォールバック (CPU を大きく食うので最後の手段)
_SW_PIPELINE = (
    "rtspsrc location={url} latency={latency} protocols=tcp ! "
    "rtph264depay ! h264parse ! {decoder} ! {rate}"
    "videoscale ! video/x-raw,width={w},height={h} ! videoconvert ! "
    "video/x-raw,format=BGR ! "
    "appsink drop=true max-buffers=1 sync=false"
)

# shm_socket 指定時。書き手は sinsei_umiusi_control の params/cameras_shm.yaml (pi_camera の分岐側)。
# 規約: caps (BGR / width / height) は書き手と一致させる。shmsrc は caps を交渉しない
_SHM_PIPELINE = (
    "shmsrc socket-path={sock} is-live=true do-timestamp=true ! "
    "video/x-raw,format=BGR,width={w},height={h},framerate=0/1 ! "
    "appsink drop=true max-buffers=1 sync=false"
)


def _live_shm_socket(path: str) -> str:
    """書き手が異常終了して socket が残ると、shmsink は <path>.0 などに作り直す。
    そのとき最も新しいものを返す (無ければ path)。"""
    def mtime(p: str) -> float:
        try:
            return os.path.getmtime(p)
        except OSError:                  # 書き手が止まって消えた
            return float("-inf")

    cands = glob.glob(glob.escape(path)) + glob.glob(glob.escape(path) + ".[0-9]*")
    return max(cands, key=mtime) if cands else path


class _Worker:
    """取得スレッド 1 本分。cv2 のオブジェクトはこのスレッドの中だけで触る。"""

    def __init__(self, gen: int):
        self.gen = gen
        self.busy = None          # (op, time.monotonic() の開始) — open / read の最中だけ
        self.thread = None

    def call(self, op, fn, *args):
        self.busy = (op, time.monotonic())
        try:
            return fn(*args)
        finally:
            self.busy = None


class CameraBridge(Node):
    def __init__(self):
        super().__init__("camera_bridge_node")
        self.declare_parameter("rtsp_url", "rtsp://localhost:8554/cam1")
        self.declare_parameter("image_topic", "/front_cam/image_raw")
        self.declare_parameter("width", 320)          # publish する幅 (autonomy.yaml の frame_w と揃える)
        self.declare_parameter("height", 240)         # publish する高さ (frame_h と揃える)
        self.declare_parameter("frame_id", "front_cam_optical")
        # 既定 0 = 制限なし。ここで絞るとフレームを取りこぼして逆に認識が落ちる。供給を減らすなら max_fps
        self.declare_parameter("max_rate_hz", 0.0)
        # デコード直後に GStreamer の中で間引く [fps]。0 = 間引かない。カメラ (cameras.yaml) は 30 fps で、
        # UI の映像と共有なのでカメラ側では下げられない。max_rate_hz と違いタイマとビートしない
        self.declare_parameter("max_fps", 15)
        self.declare_parameter("latency_ms", 100)     # rtspsrc のジッタバッファ
        self.declare_parameter("hw_decode", True)     # False -> software デコード (avdec_h264)
        # 空以外: RTSP ではなく共有メモリ (shmsrc) から読む。デコードしない。
        # 間引きは書き手側でやるので max_fps / latency_ms / hw_decode は使わない
        self.declare_parameter("shm_socket", "")
        self.declare_parameter("reconnect_sec", 3.0)  # 読めなくなったときの再接続間隔
        # open / read はワーカースレッドで呼ぶ。この秒数返らなければ ERROR を出し、
        # 取得スレッドを作り直す (止まったスレッドは殺せないので、残してよい本数に上限)
        self.declare_parameter("stall_timeout_sec", 5.0)
        self.declare_parameter("max_stalled_workers", 2)
        # --- ロギング: rosbag に残せる圧縮画像も出す (生 Image は 320x240 でも 3.5 MB/s ある) ---
        self.declare_parameter("publish_compressed", False)
        self.declare_parameter("jpeg_quality", 80)
        # 圧縮画像だけのレート上限 [Hz]。0 = 画像と同じレート。JPEG は perception と同じ
        # CPU を食うので記録目的なら間引く (全フレームは RTSP 直録側。docs/logging.md)
        self.declare_parameter("compressed_max_rate_hz", 0.0)
        # --- 自動追従: consumer_topic の実レートに publish レートを合わせる ---
        # 減るのは publish/DDS の分だけ。デコード負荷はカメラ側の framerate で決まる
        self.declare_parameter("auto_rate", False)
        self.declare_parameter("consumer_topic", "/perception_node/detections")
        self.declare_parameter("auto_rate_margin", 1.2)   # 消費レートの何倍を供給するか
        self.declare_parameter("auto_rate_min", 2.0)
        self.declare_parameter("auto_rate_max", 30.0)
        self.declare_parameter("auto_rate_step", 1.0)   # 加算増加の刻み [Hz]

        self._url = str(self.get_parameter("rtsp_url").value)
        self._shm = str(self.get_parameter("shm_socket").value)
        self._w = int(self.get_parameter("width").value)
        self._h = int(self.get_parameter("height").value)
        self._frame_id = str(self.get_parameter("frame_id").value)
        self._reconnect = float(self.get_parameter("reconnect_sec").value)
        rate = float(self.get_parameter("max_rate_hz").value)

        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self._pub = self.create_publisher(Image, str(self.get_parameter("image_topic").value), qos)
        self._bridge = CvBridge()
        self._n = 0

        self._pub_c = None
        self._c_dt = None            # 圧縮画像の間引き間隔 [s] (None = 間引かない)
        self._c_last = float("-inf")
        if bool(self.get_parameter("publish_compressed").value):
            self._pub_c = self.create_publisher(
                CompressedImage, str(self.get_parameter("image_topic").value) + "/compressed", qos)
            self._jpeg_q = int(self.get_parameter("jpeg_quality").value)
            crate = float(self.get_parameter("compressed_max_rate_hz").value)
            self._c_dt = (1.0 / crate) if crate > 0.0 else None
            self._c_last = float("-inf")
            self.get_logger().info(
                f"圧縮画像も publish します (JPEG q={self._jpeg_q}, "
                f"{f'{crate:.1f} Hz 上限' if self._c_dt else 'レート制限なし'}) — rosbag 用")

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

        # publish は時間ゲートで間引く (取得はワーカーが全フレーム読む)
        self._fixed_dt = (1.0 / rate) if rate > 0 else None
        self.add_on_set_parameters_callback(self._on_params)

        # 規約: open / read (ブロッキング) は executor の上で呼ばない。ワーカーが最新フレームを
        # _frame に置いて guard condition を叩き、executor 側は publish だけする
        self._stall_timeout = float(self.get_parameter("stall_timeout_sec").value)
        self._max_stalled = int(self.get_parameter("max_stalled_workers").value)
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._frame = None
        self._gen = 0
        self._worker = None
        self._stalled = []        # 見捨てた (open / read が返らない) スレッド
        self._frame_gc = self.create_guard_condition(self._on_frame)
        self.create_timer(min(1.0, self._stall_timeout / 2.0), self._watchdog)
        self._start_worker()

    # ------------------------------------------------------------------ capture
    def _pipeline(self, hw: bool, decimate: bool = True) -> str:
        tpl = _HW_PIPELINE if hw else _SW_PIPELINE
        fps = int(self.get_parameter("max_fps").value) if decimate else 0
        return tpl.format(
            url=self._url,
            latency=int(self.get_parameter("latency_ms").value),
            decoder="avdec_h264",
            rate=f"videorate drop-only=true max-rate={fps} ! " if fps > 0 else "",
            w=self._w, h=self._h,
        )

    def _open(self, w: _Worker):
        """ワーカースレッドから呼ぶ。開けた cap か None を返す。"""
        if self._shm:
            return self._open_shm(w)
        want_hw = bool(self.get_parameter("hw_decode").value)
        cap = w.call("open", cv2.VideoCapture, self._pipeline(want_hw), cv2.CAP_GSTREAMER)
        if not cap.isOpened() and want_hw and int(self.get_parameter("max_fps").value) > 0:
            # 間引きのせいで HW 経路がつながらないなら、software (CPU 5 倍) より間引きなしの HW を選ぶ
            self.get_logger().warning(
                "max_fps の間引き付きで HW 経路を開けません; 間引きなしの HW で開き直します",
                throttle_duration_sec=10.0)
            cap = w.call("open", cv2.VideoCapture, self._pipeline(True, decimate=False),
                         cv2.CAP_GSTREAMER)
        if not cap.isOpened() and want_hw:
            # RTSP が落ちている間は _reconnect 秒ごとにここを通るので、throttle しないと
            # 本当のエラーがログから流れてしまう (他の 2 つと同じ 10 秒に揃える)
            self.get_logger().warning(
                "ハードウェア経路 (v4l2h264dec/v4l2convert) を開けません; software に落とします "
                "(CPU 消費が 5 倍程度になります)", throttle_duration_sec=10.0)
            cap = w.call("open", cv2.VideoCapture, self._pipeline(False), cv2.CAP_GSTREAMER)
        if not cap.isOpened():
            self.get_logger().warning(
                f"GStreamer パイプラインを開けません; FFMPEG で {self._url} に直接接続します "
                "(software デコードになり CPU を食う点に注意)", throttle_duration_sec=10.0)
            cap = w.call("open", cv2.VideoCapture, self._url, cv2.CAP_FFMPEG)
        if cap.isOpened():
            self.get_logger().info(f"接続しました: {self._url} -> {self._w}x{self._h}")
            return cap
        cap.release()
        self.get_logger().error(
            f"接続できません: {self._url} (RTSP サーバとカメラは動いていますか?)",
            throttle_duration_sec=10.0)
        return None

    def _open_shm(self, w: _Worker):
        sock = _live_shm_socket(self._shm)
        cap = w.call("open", cv2.VideoCapture,
                     _SHM_PIPELINE.format(sock=sock, w=self._w, h=self._h), cv2.CAP_GSTREAMER)
        if cap.isOpened():
            self.get_logger().info(f"接続しました: shm {sock} -> {self._w}x{self._h}")
            return cap
        cap.release()
        self.get_logger().error(
            f"接続できません: shm {sock} (pi_camera を cameras_shm.yaml で起動していますか?)",
            throttle_duration_sec=10.0)
        return None

    # ------------------------------------------------------------ capture thread
    def _start_worker(self) -> None:
        with self._lock:
            self._gen += 1
            w = _Worker(self._gen)
            self._worker = w
        w.thread = threading.Thread(target=self._capture_loop, args=(w,),
                                    name=f"camera_capture_{w.gen}", daemon=True)
        w.thread.start()

    def _current(self, w: _Worker) -> bool:
        """w が今の世代か。見捨てたワーカーが後で戻ってきても結果を使わないため。"""
        return not self._stop.is_set() and w.gen == self._gen

    def _capture_loop(self, w: _Worker) -> None:
        """開く -> 読み続ける -> 失敗したら reconnect_sec 待って開き直す。"""
        cap = None
        try:
            while self._current(w):
                cap = self._open(w)
                fail = 0
                while cap is not None and self._current(w):
                    ok, frame = w.call("read", cap.read)
                    if ok and frame is not None:
                        fail = 0
                        self._put(w, frame)
                        continue
                    fail += 1
                    if fail >= 10:
                        self.get_logger().warning(
                            f"フレームが取れないので再接続します ({self._reconnect:.1f}秒間隔)",
                            throttle_duration_sec=10.0)
                        break
                if cap is not None:
                    w.call("release", cap.release)
                    cap = None
                if self._current(w):
                    # 詰めて開き直さない: test_開けない間はreconnect_secごとにだけ開き直す
                    self._stop.wait(self._reconnect)
        except Exception as e:                           # noqa: BLE001
            self.get_logger().error(f"取得スレッドが落ちました: {type(e).__name__}: {e}")
        finally:
            if cap is not None:
                cap.release()

    def _put(self, w: _Worker, frame) -> None:
        with self._lock:
            if not self._current(w):
                return
            self._frame = frame
        with contextlib.suppress(Exception):             # destroy_node と競合したとき
            self._frame_gc.trigger()

    def _watchdog(self) -> None:
        """今のワーカーの open / read が stall_timeout_sec 返らなければ作り直す。"""
        w = self._worker
        busy = w.busy if w is not None else None
        if busy is None or self._stop.is_set():
            return
        op, t0 = busy
        dt = time.monotonic() - t0
        if dt < self._stall_timeout:
            return
        self._stalled = [t for t in self._stalled if t.is_alive()]
        if len(self._stalled) >= self._max_stalled:
            self.get_logger().error(
                f"カメラの {op} が {dt:.0f} 秒返りません (書き手が詰まっている?)。"
                f"取得スレッドの作り直しは上限 {self._max_stalled} 本に達したので戻るのを待ちます",
                throttle_duration_sec=10.0)
            return
        self.get_logger().error(
            f"カメラの {op} が {dt:.0f} 秒返りません (書き手が詰まっている?); 取得スレッドを作り直します",
            throttle_duration_sec=10.0)
        self._stalled.append(w.thread)
        self._start_worker()

    # --------------------------------------------------------------------- loop
    def _on_params(self, params):
        """`ros2 param set` を実行中に効かせる (JPEG の質と圧縮の間引きだけ)。

        **受けられないものは成功を返さず理由を返す。** 黙って無視すると
        「set は通ったのに変わらない」になる (known_issues B-17)。

          * `max_rate_hz` / `stall_timeout_sec` / `max_stalled_workers` は起動時に読むだけ、
            `width`/`height`/`rtsp_url`/`hw_decode` は gst のパイプライン — **再起動でしか変えられない**
          * `jpeg_quality` / `compressed_max_rate_hz` は `publish_compressed:=true` で
            起動したときだけ意味がある
        """
        from rcl_interfaces.msg import SetParametersResult
        restart_only = ("max_rate_hz", "max_fps", "width", "height", "rtsp_url", "hw_decode",
                        "image_topic", "latency_ms", "publish_compressed", "shm_socket",
                        "stall_timeout_sec", "max_stalled_workers")
        for p in params:
            try:
                if p.name in restart_only:
                    return SetParametersResult(
                        successful=False,
                        reason=f"{p.name} は再起動でしか変えられない "
                               "(タイマ周期 / gst パイプラインの作り直しが要る)")
                if p.name in ("jpeg_quality", "compressed_max_rate_hz") and self._pub_c is None:
                    return SetParametersResult(
                        successful=False,
                        reason=f"{p.name} は publish_compressed:=true で起動したときだけ効く")
                if p.name == "jpeg_quality":
                    q = int(p.value)
                    if not 1 <= q <= 100:
                        return SetParametersResult(
                            successful=False, reason=f"jpeg_quality は 1..100: {q}")
                    self._jpeg_q = q
                    self.get_logger().warning(f"jpeg_quality={q}")
                elif p.name == "compressed_max_rate_hz":
                    r = float(p.value)
                    self._c_dt = (1.0 / r) if r > 0.0 else None
                    self.get_logger().warning(
                        f"compressed_max_rate_hz={r:.1f}"
                        + (" (制限なし)" if self._c_dt is None else ""))
            except (TypeError, ValueError) as e:            # noqa: PERF203
                return SetParametersResult(successful=False, reason=f"{p.name}: {e}")
        return SetParametersResult(successful=True)

    def _on_frame(self) -> None:
        with self._lock:
            frame, self._frame = self._frame, None
        if frame is None:
            return
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
        if self._pub_c is not None and self._compressed_due():
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

    def _compressed_due(self) -> bool:
        """圧縮画像を今フレーム出すか。cv2.imencode を呼ぶ前に判定すること —
        CPU を食うのはエンコードであって publish ではない。"""
        if self._c_dt is None:
            return True
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._c_last < self._c_dt * 0.98:   # 端数で 1 フレーム落とさないよう緩める
            return False
        self._c_last = now
        return True

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
        # cap の release はワーカーが抜けるときにやる。open / read で止まったままなら
        # daemon なので残して終了する (join で固まらない)
        self._stop.set()
        w = self._worker
        if w is not None and w.thread is not None:
            w.thread.join(timeout=2.0)
            if w.thread.is_alive():
                self.get_logger().warning("取得スレッドが止まりません (open / read が返らない); 残して終了します")
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
