"""navigator_node — high-level balloon-popping navigation, a THIN rclpy wrapper around the FSM.

Subscribes the per-frame detections (BalloonDetectionArray from perception_node) and the IMU
(sensor_msgs/Imu on /state/imu for the yaw rate), runs the shared behaviour FSM
(umiusi_perception.autonomy.BalloonBehavior — the SAME object driving tools/autonomy_run) at a fixed
control rate, and converts its {surge, heave, yaw} drive command into the four per-thruster
direct-override commands via the analytical feed-forward allocation
(umiusi_perception.control.feedforward_allocation). It publishes on the SAME direct-override topics /
message type that tools/ros_policy uses to drive the sim, so it drives the real
sinsei_umiusi_control stack UNCHANGED (sim <-> real = the hardware behind those topics).

The FSM holds the last detections between perception ticks and re-drives on them every control step,
exactly as the in-sim run does (fresh=True only on the step after a new detection message).

COMMAND MODES (command_mode parameter):
  * "setpoint" (推奨): FSM の {surge, heave, yaw} を **姿勢制御器への目標**
    (AttitudeTarget) として出し、姿勢の安定化と配分は classical_attitude /
    rl_attitude に任せる。**制御器が 1 本になる。** direct / target は配分を
    自前で持つため (1) 姿勢の安定化が一切入らない (2) 配分の実装が 2 本に割れ、
    符号のような実機固有の設定を両方へ配る必要が出る。
  * "direct" (DEFAULT): allocate here and publish per-thruster ThrusterOutput on
    /cmd/direct/... (self-enabling, bypasses core). **姿勢の安定化は入らない。**
  * "target" (EXPERIMENTAL): publish a Target on /cmd/target and let control allocate.
    NOT yet behaviour-equivalent to "direct" — validate on sim/hardware first. 2 つの
    未解決点: (1) core が POWERED-ON かつ AUTO でないと推力が出ず、stock の
    auto_target_generator が /cmd/target を取り合う (2) control 側の C++ FF は
    duty を 0.5 で clamp/slew し、第 3 象限の ESC 符号が Python 版と違う。

DEPLOY CALIBRATION (実機でしか確認できない):
  * yaw_rate_axis / yaw_rate_sign を実装済みの IMU に対して確認する (known_issues A-13)。
  * ThrusterOutput.angle は DEGREES で出す。msg コメントの [rad] が誤り (known_issues B-13)。
  * servo_sign — ch ごとの取り付け反転をここで吸収する。sim 側のアロケーションは
    4 基同符号が前提なので触らない。既定 [1,1,1,1]。

SAFETY: ~/estop または ~/arm(false) で DISARM。毎 tick detach を assert し続ける。
"""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sinsei_umiusi_msgs.msg import Target, ThrusterOutput, ThrusterRunnable
from std_srvs.srv import SetBool

from umiusi_autonomy_msgs.msg import BalloonDetectionArray
from umiusi_rl_control_msgs.msg import AttitudeTarget
from umiusi_common import thrust_sign
from umiusi_common.arm import ArmState

from umiusi_autonomy.imu_source import ImuSource

# Thruster position -> feed-forward action index. controllers.yaml: lf=id1, lb=id2, rb=id3, rf=id4;
# feedforward_allocation returns [servo_1..4, esc_1..4], so ordered positions map to indices 0..3.
# (Identical to tools/ros_policy.POSITIONS / CMD_PREFIX so the two drive the bridge the same way.)
POSITIONS = ("lf", "lb", "rb", "rf")
# 姿勢制御器の ~/setpoint と同じ QoS (latch)。実行中に繋いでも最新の目標が読める
SETPOINT_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
CMD_PREFIX = "/cmd/direct/thruster_controller/output_"


def _wrap(a):
    """角度を (-pi, pi] に畳む。"""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def _yaw_of(quat):
    """(w, x, y, z) -> yaw [rad]。quat が無ければ None。"""
    if quat is None:
        return None
    w, x, y, z = quat
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class NavigatorNode(Node):
    def __init__(self):
        super().__init__("navigator_node")
        self.declare_parameter("detections_topic", "/perception_node/detections")
        self.declare_parameter("control_hz", 50.0)
        self.declare_parameter("frame_h", 240)
        self.declare_parameter("frame_w", 320)
        self.declare_parameter("fovy_deg", 60.0)
        self.declare_parameter("servo_range_deg", 90.0)
        # ch ごとのサーボ回転センス補正。sim / アロケーション側は触らない —
        # 学習の前提なので、実機固有の事情はデプロイ境界のここで吸収する
        self.declare_parameter("servo_sign", [1.0, 1.0, 1.0, 1.0])
        # **推力の向き** (direct 経路でのみ使う)。setpoint 経路では姿勢制御器が持つ。
        # 既定は control の is_forward に合わせに行く (umiusi_common/thrust_sign.py)
        self.declare_parameter("thrust_sign", [1.0, 1.0, 1.0, 1.0])
        self.declare_parameter("thrust_sign_source", "control")
        # IMU 関連 (imu_topic / yaw_rate_axis / yaw_rate_sign / imu_max_gyro /
        # imu_max_step_deg / imu_sanity_enforce / imu_timeout) は ImuSource が宣言する。
        self.declare_parameter("publish", True)            # False = compute only, do not command
        # duty の上限。この経路には他に歯止めが無い — /cmd/direct は control の
        # max_duty もスルーレート制限も素通りする (known_issues B-12)。
        # 既定 0.25 の根拠は known_issues A-17 (rl_attitude_node と同じ値)
        self.declare_parameter("max_duty", 0.25)
        # "direct" (default, unchanged): feed-forward allocate here -> /cmd/direct ThrusterOutput.
        # "target": ride on sinsei_umiusi_control -> publish a Target on /cmd/target and let the
        # control stack allocate. EXPERIMENTAL, needs hardware/sim validation (see module docstring).
        # "setpoint" (推奨): FSM の指令を **姿勢制御器への目標** として出し、安定化と配分を
        #   classical_attitude / rl_attitude に任せる。制御器が 1 本になる
        # "direct" (既定・従来): FSM の指令を自前の配分でスラスタへ直接出す。**姿勢の
        #   安定化が入らない**。"target": control の C++ FF に載せる (こちらも安定化なし)
        self.declare_parameter("command_mode", "direct")
        self.declare_parameter("setpoint_topic", "/classical_attitude/setpoint")
        # setpoint 経路では**このノードを disarm しても機体は止まらない** — 指令を出すのを
        # やめるだけで、姿勢制御器は現在方位を保ち続ける。操縦者は「navigator を止めた =
        # 機体が止まる」と思うので、そのずれが事故になる。既定では disarm / e-stop のときに
        # **姿勢制御器も disarm しに行く**。空文字で無効 (姿勢制御器を自分で管理したいとき)
        self.declare_parameter("attitude_arm_service", "/classical_attitude/arm")
        # FSM の yaw 指令 ([-1,1]) -> 方位の変化率 [rad/s]。FSM の SEARCH_YAW=0.5 が
        # その場旋回なので、0.6 rad/s で一周およそ 20 s
        self.declare_parameter("yaw_rate_scale", 0.6)
        # 目標方位が実測方位から先行してよい上限 [rad]。既定 60 度
        self.declare_parameter("yaw_lead_max", 1.05)
        # FSM の surge/heave ([-1,1]) -> 速度指令 [m/s]
        self.declare_parameter("surge_scale", 0.35)
        self.declare_parameter("target_topic", "/cmd/target")

        self._control_hz = float(self.get_parameter("control_hz").value)
        self._dt = 1.0 / self._control_hz
        self._servo_range_deg = float(self.get_parameter("servo_range_deg").value)
        signs = [float(v) for v in self.get_parameter("servo_sign").value]
        if len(signs) != len(POSITIONS):
            # 起動時に落とす。誤った符号のまま動かすほうが危険 (ヒーブがロールに化ける)。
            raise ValueError(
                f"servo_sign needs {len(POSITIONS)} entries {POSITIONS}, got {signs}")
        self._servo_sign = signs
        self._thrust_sign = thrust_sign.resolve(
            self, POSITIONS, str(self.get_parameter("thrust_sign_source").value).strip(),
            [float(v) for v in self.get_parameter("thrust_sign").value])
        self._imu = ImuSource(self)
        self._publish = bool(self.get_parameter("publish").value)
        self._max_duty = abs(float(self.get_parameter("max_duty").value))
        self._mode = str(self.get_parameter("command_mode").value).lower()

        self._behavior = None          # lazily built (defer umiusi_perception import off the build path)
        self._alloc = None
        self._Detection = None
        self._dets = []                # last reconstructed detections (held between perception ticks)
        self._new_dets = False         # a fresh detection message arrived since the last control tick
        self._last_state = None        # FSM 状態遷移ログ用

        det_topic = self.get_parameter("detections_topic").value
        self._sub_det = self.create_subscription(
            BalloonDetectionArray, det_topic, self._on_detections, 10)
        self._imu.create_subscription()

        self._pub_setpoint = None
        self._att_arm = None
        if self._mode == "setpoint":
            sp_topic = self.get_parameter("setpoint_topic").value
            self._pub_setpoint = self.create_publisher(
                AttitudeTarget, sp_topic, SETPOINT_QOS)
            self._pub_target, self._pubs = None, {}
            self._yaw_sp = None           # 最初の IMU 到着時に現在方位へ合わせる
            svc = str(self.get_parameter("attitude_arm_service").value).strip()
            self._att_arm = self.create_client(SetBool, svc) if svc else None
            self._yaw_rate_scale = float(self.get_parameter("yaw_rate_scale").value)
            self._yaw_lead_max = abs(float(self.get_parameter("yaw_lead_max").value))
            self._surge_scale = float(self.get_parameter("surge_scale").value)
            self._heave_warned = False
            sink = f"{sp_topic} (AttitudeTarget — 安定化と配分は姿勢制御器がやる)"
        elif self._mode == "target":
            target_topic = self.get_parameter("target_topic").value
            self._pub_target = self.create_publisher(Target, target_topic, 10)
            self._pubs = {}
            sink = f"{target_topic} (Target)"
        else:
            self._pub_target = None
            self._pubs = {p: self.create_publisher(ThrusterOutput, CMD_PREFIX + p, 10)
                          for p in POSITIONS}
            sink = f"{CMD_PREFIX}{{{','.join(POSITIONS)}}}"
        self.declare_parameter("start_armed", True)    # False = launch disarmed; arm to drive
        self._arm = ArmState(self, self._detach_all,
                             start_armed=bool(self.get_parameter("start_armed").value))
        self._timer = self.create_timer(self._dt, self._control_tick)
        self.get_logger().info(
            f"navigator_node[{self._mode}]: detections='{det_topic}', imu='{self._imu.topic}' -> "
            f"{sink} @ {self._control_hz:.0f} Hz "
            f"(publish={self._publish}, max_duty={self._max_duty:.2f}, "
            f"servo_sign={self._servo_sign})")

    def _ensure_behavior(self) -> bool:
        if self._behavior is not None:
            return True
        try:
            from umiusi_perception.autonomy import BalloonBehavior
            from umiusi_perception.control import feedforward_allocation
            from umiusi_perception.balloon_detector import Detection
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(
                f"cannot import the FSM from umiusi_perception ({type(e).__name__}: {e}); "
                "is the umiusi_perception wheel installed (pip install .../packages/perception)?",
                throttle_duration_sec=10.0)
            return False
        self._behavior = BalloonBehavior(
            frame_h=int(self.get_parameter("frame_h").value),
            frame_w=int(self.get_parameter("frame_w").value),
            fovy_deg=float(self.get_parameter("fovy_deg").value),
            dt=self._dt,
        )
        self._alloc = feedforward_allocation
        self._Detection = Detection
        self.get_logger().info("behaviour FSM initialised")
        return True

    def _on_detections(self, msg: BalloonDetectionArray):
        if not self._ensure_behavior():
            return
        self._dets = [self._to_detection(d) for d in msg.detections]
        self._new_dets = True

    def _to_detection(self, d):
        return self._Detection(
            colour=d.colour,
            points=int(d.points),
            bbox=(int(d.bbox[0]), int(d.bbox[1]), int(d.bbox[2]), int(d.bbox[3])),
            centroid=(float(d.centroid[0]), float(d.centroid[1])),
            area_px=int(d.area_px),
            bearing=(float(d.azimuth), float(d.elevation)),
            range_m=float(d.range_m),
            confidence=float(d.confidence),
        )

    def _control_tick(self):
        if not self._arm.armed:            # e-stopped / disarmed: keep asserting the detach
            self._detach_all()
            return
        if not self._ensure_behavior():
            return
        self._imu.warn_if_stale()
        fresh = self._new_dets
        self._new_dets = False
        cmd, info = self._behavior.step(self._dets, self._imu.yaw_rate, heading=0.0,
                                        dt=self._dt, fresh=fresh)
        self._log_fsm(cmd, info)
        if not self._publish:
            return
        if self._mode == "setpoint":
            self._publish_setpoint(cmd)
        elif self._mode == "target":
            self._publish_target(cmd)
        else:
            # {surge, heave, yaw} -> 8-D action. Matches tools/autonomy_run: forward surge = NEGATIVE Vx,
            # heave = +Vz, yaw command on the orientation channel.
            action = self._alloc([0.0, 0.0, cmd["yaw"]], [-cmd["surge"], 0.0, cmd["heave"]])
            self._command_thrusters(action)

    def _log_fsm(self, cmd, info):
        """FSM の状態と drive 指令をログに出す。状態遷移は毎回、通常のティックは
        publish:=false のドライ確認時のみ 1 Hz。実機の通常運用ではほぼ無音。
        これが無いと publish:=false は「落ちない」ことしか確認できない (issue #18 P4)。"""
        state = info["state"]
        line = (f"FSM {state} target={info['target']} az={info['az']:+.3f} "
                f"range={info['range']:.2f} bbox={info['bbox']:.2f} blue={info['blue_threat']} "
                f"-> surge={cmd['surge']:+.3f} heave={cmd['heave']:+.3f} yaw={cmd['yaw']:+.3f}")
        if state != self._last_state:
            self._last_state = state
            self.get_logger().info(line)
        elif not self._publish:
            self.get_logger().info(line, throttle_duration_sec=1.0)

    def _publish_setpoint(self, cmd):
        """FSM の {surge, heave, yaw} を姿勢制御器への目標として出す。

        **配分も安定化もここではやらない。** direct / target の 2 経路は FSM の指令を
        自前の配分でスラスタへ直接出しており、姿勢の安定化が一切入らない — 波や推力の
        非対称で傾いても誰も戻さないし、配分の実装が 2 本に割れて符号のような設定を
        両方に配る羽目になる。この経路は姿勢制御器に預ける。

        yaw は FSM が**変化率**で出すので方位へ積分する。積分には 2 つの歯止めが要る:
          * 初期値を**実測方位**に合わせる。0 (= IMU の基準方位) から始めると、現場の
            向き次第でいきなり大きな誤差になる
          * 実測方位から `yaw_lead_max` 以上離さない。機体が追随できないまま積分すると
            誤差が 180 度まで開き、**そこが安定平衡になって出られなくなる**
            (2026-09-12 に踏んだ罠と同じ形)
        """
        yaw_now = _yaw_of(self._imu.quat)
        if yaw_now is None:
            return                        # IMU がまだ来ていない: 目標を作れない
        if self._yaw_sp is None:
            self._yaw_sp = yaw_now
            self.get_logger().info(
                f"setpoint: 目標方位を現在方位 {math.degrees(yaw_now):+.0f} deg に合わせた")
        self._yaw_sp = _wrap(self._yaw_sp + float(cmd["yaw"]) * self._yaw_rate_scale * self._dt)
        lead = _wrap(self._yaw_sp - yaw_now)
        if abs(lead) > self._yaw_lead_max:       # 目標を機体に引き戻す
            self._yaw_sp = _wrap(yaw_now + math.copysign(self._yaw_lead_max, lead))

        msg = AttitudeTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        # 水平 + 目標方位。roll/pitch は 0 (= 水平) を保たせる
        msg.orientation.w = math.cos(self._yaw_sp / 2.0)
        msg.orientation.z = math.sin(self._yaw_sp / 2.0)
        # 前進は +X。direct 経路が -cmd["surge"] を Vx に入れるのは feedforward_allocation の
        # 規約で、こちらは REP-103 body なので符号を反転しない
        msg.velocity.x = float(cmd["surge"]) * self._surge_scale
        msg.velocity.z = float(cmd["heave"]) * self._surge_scale
        self._pub_setpoint.publish(msg)

        # heave は姿勢制御器が velocity.z として受け取る。ただし鉛直の速度推定は
        # 深度センサではなく指令からの推測なので、**フィードバックは既定で切ってある**
        # (`k_v_vert=0` = 抗力に対する前進項だけ)。降下が弱いと感じたら
        # `ros2 param set /classical_attitude k_v_vert 1.2` から試す
        if not self._heave_warned and abs(float(cmd["heave"])) > 0.15:
            self._heave_warned = True
            self.get_logger().info(
                "FSM が heave を指令している。姿勢制御器は前進項だけで追う "
                "(k_v_vert=0)。追従が足りなければ k_v_vert を上げること")

    def _publish_target(self, cmd):
        # Ride on core: publish the FSM's {surge, heave, yaw} as a Target setpoint on /cmd/target and
        # let sinsei_umiusi_control's feed-forward allocation drive the thrusters. Same six numbers the
        # direct path feeds feedforward_allocation: forward surge = -velocity.x, heave = +velocity.z,
        # yaw = orientation.z (orientation.x/y and velocity.y stay 0).
        msg = Target()
        msg.orientation.z = float(cmd["yaw"])
        msg.velocity.x = float(-cmd["surge"])
        msg.velocity.z = float(cmd["heave"])
        self._pub_target.publish(msg)

    def _command_thrusters(self, action):
        # 飽和したら 4 基まとめて同じ比率で縮める。ch ごとに clip すると推力ベクトルの
        # 向きが変わる (rl_attitude_node が per-channel なのは飽和込みで学習しているため)
        peak = max(abs(float(action[4 + k])) for k in range(len(POSITIONS)))
        scale = self._max_duty / peak if peak > self._max_duty > 0.0 else 1.0
        for k, p in enumerate(POSITIONS):
            out = ThrusterOutput()
            out.runnable = ThrusterRunnable(esc=True, servo=True)
            # 符号は出口だけで掛ける (FSM と配分はモデルの座標系のまま)
            out.duty_cycle = float(action[4 + k]) * scale * self._thrust_sign[k]
            # 正規化サーボ値 -> DEGREES (受け側の規約)。範囲外は CAN フレームが送れずに
            # 落ちるだけなので、ここでハードの ±90 に収めてから出す。
            deg = float(action[k]) * self._servo_range_deg * self._servo_sign[k]
            out.angle = max(-90.0, min(90.0, deg))
            self._pubs[p].publish(out)

    def _detach_all(self):
        """DISARM / e-stop. Direct mode: zero + runnable false -> the control stack detaches
        esc/servo. Target mode: zero Target (a soft stop; the hard disarm there is core's
        power/runnable gating, which this node does not own)."""
        # **状態を戻す。** publish の有無に関わらず戻す — ドライ確認でも FSM は進むので、
        # 戻さないと次の arm が前回の途中から始まる。
        #   * 方位の積分器: disarm 中に流された分を持ち越すと再 arm の初手で大きく振れる
        #   * FSM: RAM/CONFIRM のカウンタや対象のロックが残り、突入の途中から再開する。
        #     `BalloonBehavior` に reset が無いので作り直す (次の tick で遅延生成される)
        self._yaw_sp = None
        self._behavior = None
        self._dets, self._new_dets = [], False
        if not self._publish:      # compute-only node never commands, so nothing to detach
            return
        if self._mode == "setpoint":
            # まず速度指令をゼロにする (サービスの往復を待たずに効く)
            if self._pub_setpoint is not None:
                self._publish_setpoint({"surge": 0.0, "heave": 0.0, "yaw": 0.0})
            # **そのうえで姿勢制御器も disarm する。** ここを出さないと「navigator を
            # 止めたのに機体が止まらない」— 姿勢制御器が現在方位を保ち続けるため。
            # 非同期で投げる (コールバックの中なので待てない)。届かなくても
            # 上のゼロ指令は出ているので、暴走ではなく「その場で保持」に落ちる
            if self._att_arm is not None:
                if self._att_arm.service_is_ready():
                    req = SetBool.Request()
                    req.data = False
                    self._att_arm.call_async(req)
                    self.get_logger().warning("姿勢制御器を disarm した")
                else:
                    self.get_logger().error(
                        f"姿勢制御器の arm サービスが応答しない — **機体は止まっていない。** "
                        f"手で `ros2 service call {self._att_arm.srv_name} "
                        "std_srvs/srv/SetBool '{data: false}'` を打つこと")
            return
        if self._mode == "target":
            if self._pub_target is not None:
                self._pub_target.publish(Target())
            return
        for p in POSITIONS:
            out = ThrusterOutput()
            out.runnable = ThrusterRunnable(esc=False, servo=False)
            out.duty_cycle = 0.0
            out.angle = 0.0
            self._pubs[p].publish(out)

    def stop(self):
        """Command zero / detach so the vehicle does not keep driving after we exit."""
        self._detach_all()


def main(args=None):
    rclpy.init(args=args)
    node = NavigatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.stop()
        except Exception:  # noqa: BLE001
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
