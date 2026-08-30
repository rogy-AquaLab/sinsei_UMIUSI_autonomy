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

Target mapping mirrors the direct feed-forward allocation exactly (velocity.x = -surge,
velocity.z = heave, orientation.z = yaw). See navigator_node for the standalone (no-core) drive
path and the deploy-calibration notes.
"""

from __future__ import annotations

import rclpy
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from sinsei_umiusi_msgs.msg import Target

from umiusi_autonomy_msgs.msg import BalloonDetectionArray

from umiusi_autonomy.imu_source import ImuSource


class AutoTargetGenerator(LifecycleNode):
    def __init__(self) -> None:
        super().__init__("auto_target_generator")
        self.declare_parameter("detections_topic", "/perception_node/detections")
        self.declare_parameter("target_topic", "/cmd/target")
        self.declare_parameter("control_hz", 50.0)
        self.declare_parameter("frame_h", 240)
        self.declare_parameter("frame_w", 320)
        self.declare_parameter("fovy_deg", 60.0)
        # IMU 関連 (imu_topic / yaw_rate_axis / yaw_rate_sign / imu_max_gyro /
        # imu_max_step_deg / imu_sanity_enforce / imu_timeout) は ImuSource が宣言する。
        # navigator_node と同じ扱いを 1 箇所に寄せてある (issue #19-5)。
        self._imu = ImuSource(self)

        self._dt = 1.0 / float(self.get_parameter("control_hz").value)

        self._behavior = None          # lazily built (defer the umiusi_perception import off the build path)
        self._Detection = None
        self._dets = []                # last reconstructed detections (held between perception ticks)
        self._new_dets = False         # a fresh detection message arrived since the last control tick
        self._pub = None
        self._sub_det = None
        self._timer = None

    # ---- lifecycle transitions ----
    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self._pub = self.create_publisher(Target, self.get_parameter("target_topic").value, 10)
        self._sub_det = self.create_subscription(
            BalloonDetectionArray, self.get_parameter("detections_topic").value, self._on_detections, 10)
        self._imu.create_subscription()
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
        self._imu.destroy()
        if self._pub is not None:
            self.destroy_publisher(self._pub)
        self._timer = self._sub_det = self._pub = None

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
        self._imu.warn_if_stale()
        fresh = self._new_dets
        self._new_dets = False
        cmd, _info = self._behavior.step(self._dets, self._imu.yaw_rate, heading=0.0,
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
