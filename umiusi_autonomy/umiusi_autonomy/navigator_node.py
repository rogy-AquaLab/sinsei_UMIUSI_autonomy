"""navigator_node — high-level balloon-popping navigation, a THIN rclpy wrapper around the FSM.

Subscribes the per-frame detections (``BalloonDetectionArray`` from ``perception_node``) and the IMU
(``sinsei_umiusi_msgs/ImuState`` for the yaw rate), runs the shared behaviour FSM
(``umiusi_perception.autonomy.BalloonBehavior`` — the SAME object driving ``tools/autonomy_run``) at a fixed
control rate, and converts its {surge, heave, yaw} drive command into the four per-thruster
direct-override commands via the analytical feed-forward allocation
(``umiusi_perception.control.feedforward_allocation``). It publishes on the SAME direct-override topics /
message type that ``tools/ros_policy`` uses to drive the sim, so it drives the real
sinsei_umiusi_control stack UNCHANGED (sim <-> real = the hardware behind those topics).

The FSM holds the last detections between perception ticks and re-drives on them every control step,
exactly as the in-sim run does (``fresh=True`` only on the step after a new detection message).

COMMAND MODES (``command_mode`` parameter):
  * ``"direct"`` (DEFAULT — unchanged behaviour): allocate here and publish per-thruster
    ``ThrusterOutput`` on ``/cmd/direct/...`` (self-enabling, bypasses core).
  * ``"target"`` (EXPERIMENTAL — "ride on core"): publish a ``sinsei_umiusi_msgs/Target``
    (velocity + orientation) on ``/cmd/target`` and let ``sinsei_umiusi_control`` allocate, so
    autonomy plugs into the existing core power/mode pipeline instead of overriding thrusters.
    The FSM's {surge, heave, yaw} maps to Target exactly as it feeds ``feedforward_allocation``
    (velocity.x=-surge, velocity.z=heave, orientation.z=yaw). NOT yet behaviour-equivalent to
    ``"direct"`` — validate on sim/hardware first. Known control-side gaps to reconcile:
      1. core must be POWERED-ON and in AUTO (a Target alone does not enable thrust — the
         ``/cmd/thruster_runnable_all`` flag from core's AUTO node does), and the stock
         ``auto_target_generator`` placeholder must be replaced/stopped or it races on /cmd/target.
      2. sinsei_umiusi_control's C++ feed-forward emits servo in DEGREES and clamps/slews ESC duty
         (max 0.5), and its ESC thrust-sign differs from the Python port in the third force quadrant
         — so magnitudes/signs can diverge from the direct path until those are reconciled.

DEPLOY CALIBRATION (verify on hardware, cannot be inferred from the sim):
  * ImuState.angular_velocity is in DEG/S; the sim FSM expects the body yaw rate in RAD/S about the
    vehicle's vertical axis. ``yaw_rate_axis`` / ``yaw_rate_sign`` select and orient that component
    (default y-up, +, matching the sim). Confirm the axis/sign against the mounted IMU.
  * ThrusterOutput.angle is documented in RAD; ``servo_range_deg`` sets the half-range used to map
    the normalised servo action to radians (default 90, matching configs/umiusi.yaml). NOTE:
    tools/ros_policy currently scales in degrees — reconcile the two against the live bridge during
    hardware bring-up (this is the spec's open "FF-frame sign reconcile" item).

SAFETY: ``~/estop`` (std_msgs/Bool, true) or ``~/arm`` (std_srvs/SetBool, data:false) DISARMs — the
control tick stops and asserts a detach every cycle (direct mode: runnable esc/servo = false + zero;
target mode: zero Target). Re-arm via ``~/arm`` (data:true) or ``~/estop`` (false). ``start_armed``.
"""

from __future__ import annotations

import math

import rclpy
from rclpy.node import Node
from sinsei_umiusi_msgs.msg import Target, ThrusterOutput, ThrusterRunnable

from umiusi_autonomy_msgs.msg import BalloonDetectionArray
from umiusi_rl_control.arm import ArmState

# Thruster position -> feed-forward action index. controllers.yaml: lf=id1, lb=id2, rb=id3, rf=id4;
# feedforward_allocation returns [servo_1..4, esc_1..4], so ordered positions map to indices 0..3.
# (Identical to tools/ros_policy.POSITIONS / CMD_PREFIX so the two drive the bridge the same way.)
POSITIONS = ("lf", "lb", "rb", "rf")
CMD_PREFIX = "/cmd/direct/thruster_controller/output_"
_AXIS = {"x": 0, "y": 1, "z": 2}


class NavigatorNode(Node):
    def __init__(self):
        super().__init__("navigator_node")
        self.declare_parameter("detections_topic", "/perception_node/detections")
        self.declare_parameter("imu_topic", "/state/imu_state")
        self.declare_parameter("control_hz", 50.0)
        self.declare_parameter("frame_h", 240)
        self.declare_parameter("frame_w", 320)
        self.declare_parameter("fovy_deg", 60.0)
        self.declare_parameter("servo_range_deg", 90.0)
        self.declare_parameter("yaw_rate_axis", "y")      # IMU axis carrying the vehicle yaw rate
        self.declare_parameter("yaw_rate_sign", 1.0)
        self.declare_parameter("publish", True)            # False = compute only, do not command
        # "direct" (default, unchanged): feed-forward allocate here -> /cmd/direct ThrusterOutput.
        # "target": ride on sinsei_umiusi_control -> publish a Target on /cmd/target and let the
        # control stack allocate. EXPERIMENTAL, needs hardware/sim validation (see module docstring).
        self.declare_parameter("command_mode", "direct")
        self.declare_parameter("target_topic", "/cmd/target")

        self._control_hz = float(self.get_parameter("control_hz").value)
        self._dt = 1.0 / self._control_hz
        self._servo_range_rad = math.radians(float(self.get_parameter("servo_range_deg").value))
        self._yaw_axis = _AXIS.get(str(self.get_parameter("yaw_rate_axis").value).lower(), 1)
        self._yaw_sign = float(self.get_parameter("yaw_rate_sign").value)
        self._publish = bool(self.get_parameter("publish").value)
        self._mode = str(self.get_parameter("command_mode").value).lower()

        self._behavior = None          # lazily built (defer umiusi_perception import off the build path)
        self._alloc = None
        self._Detection = None
        self._dets = []                # last reconstructed detections (held between perception ticks)
        self._new_dets = False         # a fresh detection message arrived since the last control tick
        self._yaw_rate = 0.0

        det_topic = self.get_parameter("detections_topic").value
        imu_topic = self.get_parameter("imu_topic").value
        self._sub_det = self.create_subscription(
            BalloonDetectionArray, det_topic, self._on_detections, 10)
        # Import ImuState lazily-safe: it is a build dep so importing at module top is fine here.
        from sinsei_umiusi_msgs.msg import ImuState
        self._sub_imu = self.create_subscription(ImuState, imu_topic, self._on_imu, 10)

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
            f"navigator_node[{self._mode}]: detections='{det_topic}', imu='{imu_topic}' -> "
            f"{sink} @ {self._control_hz:.0f} Hz (publish={self._publish})")

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

    def _on_imu(self, msg):
        # ImuState.angular_velocity is DEG/S; the FSM wants the body yaw rate in RAD/S.
        v = (msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z)
        self._yaw_rate = self._yaw_sign * math.radians(v[self._yaw_axis])

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
        fresh = self._new_dets
        self._new_dets = False
        cmd, _info = self._behavior.step(self._dets, self._yaw_rate, heading=0.0,
                                         dt=self._dt, fresh=fresh)
        if not self._publish:
            return
        if self._mode == "target":
            self._publish_target(cmd)
        else:
            # {surge, heave, yaw} -> 8-D action. Matches tools/autonomy_run: forward surge = NEGATIVE Vx,
            # heave = +Vz, yaw command on the orientation channel.
            action = self._alloc([0.0, 0.0, cmd["yaw"]], [-cmd["surge"], 0.0, cmd["heave"]])
            self._command_thrusters(action)

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
        for k, p in enumerate(POSITIONS):
            out = ThrusterOutput()
            out.runnable = ThrusterRunnable(esc=True, servo=True)
            out.duty_cycle = float(action[4 + k])              # esc command in [-1, 1]
            out.angle = float(action[k]) * self._servo_range_rad  # normalised servo -> radians
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
