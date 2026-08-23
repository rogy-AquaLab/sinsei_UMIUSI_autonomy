"""auto_target_generator — AUTO-mode target source for sinsei_umiusi_core, driven by the FSM.

A drop-in **lifecycle** replacement for core's placeholder ``auto_target_generator``: same node name
and lifecycle contract, so core's behaviour tree activates/deactivates it via
``/auto_target_generator/change_state`` when entering/leaving AUTO. Instead of empty Targets it runs
the shared balloon-popping FSM (``umiusi_perception.autonomy.BalloonBehavior`` — the SAME object as
``tools/autonomy_run`` and ``navigator_node``) and publishes its ``{surge, heave, yaw}`` command as a
``sinsei_umiusi_msgs/Target`` on ``/cmd/target``; ``sinsei_umiusi_control`` does the allocation.

This is how autonomy "rides on core": power / mode / thruster-enable stay in core's hands (a Target
alone does not move thrusters — core's AUTO node also publishes the runnable flag, and power must be
on); this node only produces the setpoint while its lifecycle is active. Perception + FSM are the
ROS-free ``umiusi_perception`` code, so behaviour is identical to the in-sim run.

Target mapping mirrors the direct feed-forward allocation exactly (velocity.x = -surge,
velocity.z = heave, orientation.z = yaw). See ``navigator_node`` for the standalone (no-core) drive
path and the deploy-calibration notes.
"""

from __future__ import annotations

import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from sensor_msgs.msg import Imu
from sinsei_umiusi_msgs.msg import Target
from umiusi_rl_control.imu_sanity import ImuSanity

from umiusi_autonomy_msgs.msg import BalloonDetectionArray

_AXIS = {"x": 0, "y": 1, "z": 2}


class AutoTargetGenerator(LifecycleNode):
    def __init__(self) -> None:
        super().__init__("auto_target_generator")
        self.declare_parameter("detections_topic", "/perception_node/detections")
        self.declare_parameter("imu_topic", "/state/imu")
        self.declare_parameter("target_topic", "/cmd/target")
        self.declare_parameter("control_hz", 50.0)
        self.declare_parameter("frame_h", 240)
        self.declare_parameter("frame_w", 320)
        self.declare_parameter("fovy_deg", 60.0)
        self.declare_parameter("yaw_rate_axis", "z")      # IMU axis carrying the vehicle yaw rate (REP-103: z)
        self.declare_parameter("yaw_rate_sign", 1.0)
        # IMU のサニティフィルタ (実機の化けサンプル対策)。0 以下で無効化できる。
        self.declare_parameter("imu_max_gyro", 10.0)        # [rad/s] 検出の閾値
        self.declare_parameter("imu_max_step_deg", 30.0)    # 1 サンプルの姿勢跳躍上限 [deg]
        # 既定は「検出するが破棄しない」(rl_attitude_node と同じ理由)
        self.declare_parameter("imu_sanity_enforce", False)

        self._dt = 1.0 / float(self.get_parameter("control_hz").value)
        self._yaw_axis = _AXIS.get(str(self.get_parameter("yaw_rate_axis").value).lower(), 1)
        self._yaw_sign = float(self.get_parameter("yaw_rate_sign").value)
        self._imu_sanity = ImuSanity(
            max_gyro=float(self.get_parameter("imu_max_gyro").value),
            max_step_deg=float(self.get_parameter("imu_max_step_deg").value),
            enforce=bool(self.get_parameter("imu_sanity_enforce").value))

        self._behavior = None          # lazily built (defer the umiusi_perception import off the build path)
        self._Detection = None
        self._dets = []                # last reconstructed detections (held between perception ticks)
        self._new_dets = False         # a fresh detection message arrived since the last control tick
        self._yaw_rate = 0.0
        self._pub = None
        self._sub_det = None
        self._sub_imu = None
        self._timer = None

    # ---- lifecycle transitions ----
    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._pub = self.create_publisher(Target, self.get_parameter("target_topic").value, 10)
        self._sub_det = self.create_subscription(
            BalloonDetectionArray, self.get_parameter("detections_topic").value, self._on_detections, 10)
        self._sub_imu = self.create_subscription(
            Imu, self.get_parameter("imu_topic").value, self._on_imu, 10)
        self._timer = self.create_timer(self._dt, self._tick, autostart=False)
        self.get_logger().info("auto_target_generator configured (FSM-driven Target on /cmd/target)")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._timer.reset()
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._timer.cancel()
        if self._pub is not None:
            self._pub.publish(Target())    # zero setpoint on leaving AUTO
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
        if self._sub_imu is not None:
            self.destroy_subscription(self._sub_imu)
        if self._pub is not None:
            self.destroy_publisher(self._pub)
        self._timer = self._sub_det = self._sub_imu = self._pub = None

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
        self.get_logger().info("behaviour FSM initialised")
        return True

    def _on_imu(self, msg) -> None:
        # 実機の BNO055 は物理的にありえないサンプルを混ぜてくる (ゼロクォータニオン、
        # 角速度の int16 フルスケール張り付き、姿勢の跳躍)。ヨーレートをそのまま制御に
        # 使うので、1 発のスパイクで制御が跳ねる。ただし **既定では検出するだけで弾かない**
        # (`imu_sanity_enforce`)。理由は imu_sanity.py 冒頭。
        q, g = msg.orientation, msg.angular_velocity
        sample, reason = self._imu_sanity.update((q.w, q.x, q.y, q.z), (g.x, g.y, g.z))
        if reason is not None:
            self.get_logger().warning(
                self._imu_sanity.describe(reason),
                throttle_duration_sec=5.0)
            if sample is None:
                return          # まだ 1 つも有効値が無い
        # sensor_msgs/Imu.angular_velocity is RAD/S (ROS standard), which is what the FSM wants.
        self._yaw_rate = self._yaw_sign * sample.gyro[self._yaw_axis]

    def _on_detections(self, msg: BalloonDetectionArray) -> None:
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

    def _tick(self) -> None:
        if not self._ensure_behavior():
            return
        fresh = self._new_dets
        self._new_dets = False
        cmd, _info = self._behavior.step(self._dets, self._yaw_rate, heading=0.0,
                                         dt=self._dt, fresh=fresh)
        # {surge, heave, yaw} -> Target, the same six numbers the direct path feeds
        # feedforward_allocation: forward surge = -velocity.x, heave = +velocity.z, yaw = orientation.z.
        msg = Target()
        msg.orientation.z = float(cmd["yaw"])
        msg.velocity.x = float(-cmd["surge"])
        msg.velocity.z = float(cmd["heave"])
        self._pub.publish(msg)


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
