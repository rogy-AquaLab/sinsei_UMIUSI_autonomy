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
  * "direct" (DEFAULT): allocate here and publish per-thruster ThrusterOutput on
    /cmd/direct/... (self-enabling, bypasses core).
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

import rclpy
from rclpy.node import Node
from sinsei_umiusi_msgs.msg import Target, ThrusterOutput, ThrusterRunnable

from umiusi_autonomy_msgs.msg import BalloonDetectionArray
from umiusi_common.arm import ArmState

from umiusi_autonomy.imu_source import ImuSource

# Thruster position -> feed-forward action index. controllers.yaml: lf=id1, lb=id2, rb=id3, rf=id4;
# feedforward_allocation returns [servo_1..4, esc_1..4], so ordered positions map to indices 0..3.
# (Identical to tools/ros_policy.POSITIONS / CMD_PREFIX so the two drive the bridge the same way.)
POSITIONS = ("lf", "lb", "rb", "rf")
CMD_PREFIX = "/cmd/direct/thruster_controller/output_"


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
        self.declare_parameter("command_mode", "direct")
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

        if self._mode == "target":
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
        if self._mode == "target":
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
            out.duty_cycle = float(action[4 + k]) * scale      # esc command in [-1, 1]
            # 正規化サーボ値 -> DEGREES (受け側の規約)。範囲外は CAN フレームが送れずに
            # 落ちるだけなので、ここでハードの ±90 に収めてから出す。
            deg = float(action[k]) * self._servo_range_deg * self._servo_sign[k]
            out.angle = max(-90.0, min(90.0, deg))
            self._pubs[p].publish(out)

    def _detach_all(self):
        """DISARM / e-stop. Direct mode: zero + runnable false -> the control stack detaches
        esc/servo. Target mode: zero Target (a soft stop; the hard disarm there is core's
        power/runnable gating, which this node does not own)."""
        if not self._publish:      # compute-only node never commands, so nothing to detach
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
