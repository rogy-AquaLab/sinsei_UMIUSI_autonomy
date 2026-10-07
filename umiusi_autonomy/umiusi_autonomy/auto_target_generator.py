"""auto_target_generator — AUTO-mode target source for sinsei_umiusi_core, driven by the FSM.

A drop-in lifecycle replacement for core's placeholder auto_target_generator: same node name
and lifecycle contract, so core's behaviour tree activates/deactivates it via
/auto_target_generator/change_state when entering/leaving AUTO. Instead of empty Targets it runs
the shared balloon-popping FSM (umiusi_perception.autonomy.BalloonBehavior — the SAME object as
tools/autonomy_run and navigator_node) and publishes its {surge, heave, yaw} command as a
sinsei_umiusi_msgs/Target on /cmd/target; sinsei_umiusi_control does the allocation.

This is how autonomy "rides on core": power / mode / thruster-enable stay in core's hands (a Target
alone does not move thrusters — core's AUTO node also publishes the runnable flag, and power must be
on); this node only produces the setpoint while its lifecycle is active. Perception + FSM are the
ROS-free umiusi_perception code, so behaviour is identical to the in-sim run.

Target mapping (control dev-0921 interface, see to_control_setpoint):
  Target.velocity          normalized command [-1, 1]; x = surge_sign * surge, z = heave
  AttitudeTarget           level attitude, yaw_rate = yaw_rate_scale * yaw [rad/s], hold_yaw false
surge_sign follows the UI gamepad (stick forward -> +x), which drives the same field.
"""

from __future__ import annotations

import time

import rclpy
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from sinsei_umiusi_msgs.msg import AttitudeTarget, Target

from umiusi_autonomy_msgs.msg import BalloonDetectionArray

from umiusi_autonomy import fsm_params
from umiusi_autonomy.imu_source import ImuSource


def to_control_setpoint(cmd: dict, surge_sign: float, yaw_rate_scale: float):
    """FSM の {surge, heave, yaw} -> (velocity.x, velocity.z, yaw_rate [rad/s])。"""
    def clip(v: float) -> float:
        return max(-1.0, min(1.0, float(v)))
    return (clip(surge_sign * cmd["surge"]), clip(cmd["heave"]),
            yaw_rate_scale * clip(cmd["yaw"]))


def live_detections(dets: list, last_rx: float | None, now: float, timeout_s: float) -> list:
    """最後に検出メッセージが届いてから timeout_s を超えたら空を返す。

    カメラや認識が止まると検出メッセージが来なくなる。最後の検出を握ったまま FSM に渡すと、
    トラッカーは同じ検出で見失い回数を 0 に戻すので、居ない風船を追い続ける (B-24)。
    timeout_s <= 0 で無効 (握り続ける)。
    """
    if last_rx is None:
        return []
    if timeout_s > 0.0 and now - last_rx > timeout_s:
        return []
    return dets


def neutral_attitude_target() -> AttitudeTarget:
    msg = AttitudeTarget()
    msg.attitude.w = 1.0
    return msg


class AutoTargetGenerator(LifecycleNode):
    def __init__(self) -> None:
        super().__init__("auto_target_generator")
        self.declare_parameter("detections_topic", "/perception_node/detections")
        self.declare_parameter("target_topic", "/cmd/target")
        self.declare_parameter("attitude_target_topic", "/cmd/attitude_target")
        # UI のゲームパッドと同じ規約 (前に倒す -> +x)。実機で前進が逆なら -1
        self.declare_parameter("surge_sign", 1.0)
        # FSM の yaw [-1, 1] -> rad/s。UI のスティック最大 (MAX_YAW_RATE) と同じ
        self.declare_parameter("yaw_rate_scale", 1.0)
        self.declare_parameter("control_hz", 50.0)
        # 検出がこれだけ途切れたら「何も見えていない」とみなす [s]。perception は最大 10 Hz
        self.declare_parameter("detections_timeout_s", 0.5)
        # FSM の調整値 fsm.* は FSM を作るとき (configure) に宣言する。値は config/competition.yaml
        self.declare_parameter("frame_h", 240)
        self.declare_parameter("frame_w", 320)
        self.declare_parameter("fovy_deg", 60.0)
        # IMU 関連のパラメータは ImuSource が宣言する (navigator_node と共通)
        self._imu = ImuSource(self)

        self._dt = 1.0 / float(self.get_parameter("control_hz").value)

        self._behavior = None          # lazily built (defer the umiusi_perception import off the build path)
        self._Detection = None
        self._dets = []                # last reconstructed detections (held between perception ticks)
        self._new_dets = False         # a fresh detection message arrived since the last control tick
        self._last_det_rx = None       # time.monotonic() of the last detection message
        self._last_tick = None         # time.monotonic() of the previous control tick
        self._pub = None
        self._pub_att = None
        self._sub_det = None
        self._timer = None

    # ---- lifecycle transitions ----
    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._pub = self.create_publisher(Target, self.get_parameter("target_topic").value, 10)
        self._pub_att = self.create_publisher(
            AttitudeTarget, self.get_parameter("attitude_target_topic").value, 10)
        self._sub_det = self.create_subscription(
            BalloonDetectionArray, self.get_parameter("detections_topic").value, self._on_detections, 10)
        self._imu.create_subscription()
        self._timer = self.create_timer(self._dt, self._tick, autostart=False)
        self._ensure_behavior()        # fsm.* を AUTO の前から `ros2 param set` できるように
        self.get_logger().info("auto_target_generator configured (FSM-driven Target on /cmd/target)")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._last_tick = None
        self._timer.reset()
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._timer.cancel()
        if self._pub is not None:
            self._pub.publish(Target())    # zero setpoint on leaving AUTO
        if self._pub_att is not None:
            self._pub_att.publish(neutral_attitude_target())
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._destroy()
        return TransitionCallbackReturn.SUCCESS

    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._destroy()
        return TransitionCallbackReturn.SUCCESS

    def _destroy(self) -> None:
        if self._timer is not None:
            self.destroy_timer(self._timer)
        if self._sub_det is not None:
            self.destroy_subscription(self._sub_det)
        self._imu.destroy()
        if self._pub is not None:
            self.destroy_publisher(self._pub)
        if self._pub_att is not None:
            self.destroy_publisher(self._pub_att)
        self._timer = self._sub_det = self._pub = self._pub_att = None

    # ---- FSM plumbing (mirrors navigator_node) ----
    def _ensure_behavior(self) -> bool:
        if self._behavior is not None:
            return True
        try:
            from umiusi_perception.autonomy import BalloonBehavior
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
        self._Detection = Detection
        self._declare_fsm_params()
        self.get_logger().info("behaviour FSM initialised")
        return True

    def _declare_fsm_params(self) -> None:
        """fsm.* を宣言して FSM に入れる。既定は FSM が今持っている値、yaml にあればそちらが勝つ。"""
        absent = set(fsm_params.missing(self._behavior))
        if absent:
            self.get_logger().warn(
                f"この umiusi_perception の FSM に無い調整値 (古い wheel): {sorted(absent)}。これらは効かない")
        self.add_on_set_parameters_callback(self._on_set_fsm_params)
        for p in fsm_params.FSM_PARAMS:
            if p.name in absent:
                continue
            # 型は固定しない: yaml の 5 と 5.0 の違いで configure が落ちないように (値は apply が変換する)
            value = self.declare_parameter(
                fsm_params.PREFIX + p.name, fsm_params.current_value(self._behavior, p),
                ParameterDescriptor(dynamic_typing=True)).value
            fsm_params.apply(self._behavior, p, value)

    def _on_set_fsm_params(self, params):
        """`ros2 param set /auto_target_generator fsm.<名前> <値>` を走らせたまま効かせる。"""
        from rcl_interfaces.msg import SetParametersResult
        for param in params:
            p = fsm_params.by_ros_name(param.name)
            if p is None:
                continue
            try:
                fsm_params.apply(self._behavior, p, param.value)
            except (TypeError, ValueError) as e:
                return SetParametersResult(successful=False, reason=f"{param.name}: {e}")
        return SetParametersResult(successful=True)

    def _on_detections(self, msg: BalloonDetectionArray) -> None:
        if not self._ensure_behavior():
            return
        self._dets = [self._to_detection(d) for d in msg.detections]
        self._new_dets = True
        self._last_det_rx = time.monotonic()

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

    def _tick(self) -> None:
        if not self._ensure_behavior():
            return
        self._imu.warn_if_stale()
        now = time.monotonic()
        dt = fsm_params.measured_dt(self._last_tick, now, self._dt)
        self._last_tick = now
        # BalloonBehavior の探索は step() の dt ではなく self.dt で積分するので、両方に入れる
        self._behavior.dt = dt
        fresh = self._new_dets
        self._new_dets = False
        dets = live_detections(self._dets, self._last_det_rx, time.monotonic(),
                               float(self.get_parameter("detections_timeout_s").value))
        if self._dets and not dets:
            self.get_logger().warn(
                "検出が途切れた (カメラ / 認識が止まっている?)。何も見えていないものとして扱う",
                throttle_duration_sec=5.0)
        cmd, _info = self._behavior.step(dets, self._imu.yaw_rate, heading=0.0,
                                         dt=dt, fresh=fresh)
        vx, vz, yaw_rate = to_control_setpoint(
            cmd, float(self.get_parameter("surge_sign").value),
            float(self.get_parameter("yaw_rate_scale").value))
        msg = Target()
        msg.velocity.x = vx
        msg.velocity.z = vz
        self._pub.publish(msg)
        att = neutral_attitude_target()
        att.header.stamp = self.get_clock().now().to_msg()
        att.yaw_rate = yaw_rate
        self._pub_att.publish(att)


def main(args=None):
    rclpy.init(args=args)
    node = AutoTargetGenerator()
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
