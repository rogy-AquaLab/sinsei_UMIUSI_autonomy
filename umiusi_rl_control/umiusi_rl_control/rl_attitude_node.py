"""rl_attitude_node — drive the thrusters with a trained RL attitude(-velocity) policy.

A SELF-CONTAINED rclpy port of ``umiusi_sim/tools/ros_policy.py``: it needs NO umiusi_sim / umiusi_rl /
mujoco — only the bundled policy (``models/cruise_policy``) and stable-baselines3 + torch + gymnasium
to run it. Loop:

  * SUBSCRIBE  /state/imu (sensor_msgs/Imu) + /state/thruster_state_all (ThrusterStateAll)
  * rebuild the policy's 25-D observation for task=attitude_velocity / obs_mode=imu, EXACTLY as
    ``UmiusiPoseEnv._get_obs`` does — layout [ori_err(3), gyro(3), v_cmd(3), servo_n(4), thrust_n(4),
    prev_action(8)] — using a vendored ``mju_subQuat`` (verified bit-identical to MuJoCo), apply the
    training-time VecNormalize, and ``policy.predict``.
  * PUBLISH the four direct-override /cmd/direct/thruster_controller/output_{lf,lb,rb,rf}
    (ThrusterOutput, runnable=true), action [servo x4, esc x4] -> {angle, duty_cycle}.

Command: defaults to hold UPRIGHT (target = identity) + CRUISE forward (body +X at ``vel_cmd`` m/s),
and can be overridden in REAL TIME by publishing an ``umiusi_rl_control_msgs/AttitudeTarget`` on
``setpoint_topic`` (target attitude quaternion + feed-forward velocity in the target-body frame;
``type_mask`` selects which fields apply). Last message wins; a teleop / joystick controller just
publishes it.

SAFETY: ``~/estop`` (std_msgs/Bool, true) or ``~/arm`` (std_srvs/SetBool, data:false) DISARMs
immediately — the loop stops predicting and asserts a DETACH every tick (ThrusterOutput runnable
esc/servo = false, zero output), so the control stack releases the thrusters. Re-arm with the
``~/arm`` service (data:true). ``start_armed:=false`` launches disarmed.

UNIT CAVEATS (inherited from ros_policy; confirm on the live bridge — the spec's open
"FF-frame reconcile" item):
  * IMU ``angular_velocity`` is used as-is as rad/s (sensor_msgs/Imu is rad/s by the ROS standard, so
    ``gyro_deg_per_sec`` stays False; set it true only if a bridge wrongly sends deg/s).
  * servo output ``ThrusterOutput.angle`` is published in DEGREES (= action * servo_range_deg), as
    ros_policy does; msg documents rad. Confirm what the plugin/hardware expects.
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import Imu
from sinsei_umiusi_msgs.msg import ThrusterOutput, ThrusterRunnable, ThrusterStateAll

from umiusi_rl_control.arm import ArmState
from umiusi_rl_control_msgs.msg import AttitudeTarget

POSITIONS = ("lf", "lb", "rb", "rf")
CMD_PREFIX = "/cmd/direct/thruster_controller/output_"
OBS_DIM = 25
ACT_DIM = 8


def _quat_mul(a, b):
    w0, x0, y0, z0 = a
    w1, x1, y1, z1 = b
    return np.array([
        w0 * w1 - x0 * x1 - y0 * y1 - z0 * z1,
        w0 * x1 + x0 * w1 + y0 * z1 - z0 * y1,
        w0 * y1 - x0 * z1 + y0 * w1 + z0 * x1,
        w0 * z1 + x0 * y1 - y0 * x1 + z0 * w1,
    ])


def mju_sub_quat(qa, qb):
    """Rotation 3-vector taking ``qb`` to ``qa`` — a numpy reimplementation of MuJoCo's
    ``mju_subQuat`` (verified bit-identical), so we avoid a mujoco dependency on the robot."""
    qn = np.array([qb[0], -qb[1], -qb[2], -qb[3]])   # conjugate (unit quat)
    qd = _quat_mul(qn, qa)
    v = qd[1:4].astype(float)
    sin_a_2 = np.linalg.norm(v)
    if sin_a_2 < 1e-12:
        return np.zeros(3)
    speed = 2.0 * math.atan2(sin_a_2, qd[0])
    if speed > math.pi:      # larger-than-pi rotation is the opposite direction
        speed -= 2.0 * math.pi
    return (v / sin_a_2) * speed


class RlAttitudeNode(Node):
    def __init__(self):
        super().__init__("rl_attitude_node")
        self.declare_parameter("model_path", "")   # "" -> bundled models/cruise_policy/final.zip
        self.declare_parameter("imu_topic", "/state/imu")
        self.declare_parameter("thruster_state_topic", "/state/thruster_state_all")
        self.declare_parameter("control_hz", 50.0)
        self.declare_parameter("vel_cmd", 0.4)             # forward (+X) commanded speed [m/s]
        self.declare_parameter("servo_range_deg", 90.0)
        self.declare_parameter("gyro_deg_per_sec", False)  # convert IMU gyro deg/s -> rad/s (see caveats)
        self.declare_parameter("publish", True)            # False = predict only, do not command
        self.declare_parameter("start_armed", True)        # False = launch disarmed (safe); arm to run
        # Real-time setpoint (hold last; until a message arrives, use the launch defaults below).
        self.declare_parameter("setpoint_topic", "~/setpoint")   # umiusi_rl_control_msgs/AttitudeTarget

        self._hz = float(self.get_parameter("control_hz").value)
        self._dt = 1.0 / self._hz
        self._vel = float(self.get_parameter("vel_cmd").value)
        self._servo_range_deg = float(self.get_parameter("servo_range_deg").value)
        self._servo_range_rad = math.radians(self._servo_range_deg)
        self._gyro_to_rad = bool(self.get_parameter("gyro_deg_per_sec").value)
        self._publish = bool(self.get_parameter("publish").value)

        self._target_quat = np.array([1.0, 0.0, 0.0, 0.0])   # identity = hold upright/level
        self._v_cmd = np.array([self._vel, 0.0, 0.0])         # cruise along body +X
        self._prev_action = np.zeros(ACT_DIM)
        self._model = None
        self._norm_obs = None
        self._imu = None
        self._thr = None

        self._sub_imu = self.create_subscription(
            Imu, self.get_parameter("imu_topic").value, self._on_imu, 1)
        self._sub_thr = self.create_subscription(
            ThrusterStateAll, self.get_parameter("thruster_state_topic").value, self._on_thr, 1)
        # Real-time setpoint (optional): last message wins; absence keeps the defaults above.
        sp_topic = self.get_parameter("setpoint_topic").value
        self._sub_sp = self.create_subscription(AttitudeTarget, sp_topic, self._on_setpoint, 1)
        self._pubs = {p: self.create_publisher(ThrusterOutput, CMD_PREFIX + p, 10) for p in POSITIONS}
        self._arm = ArmState(self, self._detach_all,
                             start_armed=bool(self.get_parameter("start_armed").value))
        self._timer = self.create_timer(self._dt, self._tick)
        self.get_logger().info(
            f"rl_attitude_node: default target=upright v_cmd=[{self._vel:.3f},0,0] m/s @ {self._hz:.0f} Hz "
            f"(publish={self._publish}); live setpoint (AttitudeTarget) on '{sp_topic}'; "
            "loading policy on first state...")

    def _on_imu(self, msg):
        self._imu = msg

    def _on_thr(self, msg):
        self._thr = msg

    def _on_setpoint(self, msg):
        # umiusi_rl_control_msgs/AttitudeTarget. type_mask selects which fields to apply (mavros-style):
        # a masked-out field keeps its previous value; default mask 0 = update both.
        if not (msg.type_mask & AttitudeTarget.IGNORE_ATTITUDE):
            q = np.array([msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z],
                         dtype=float)
            n = np.linalg.norm(q)          # ROS xyzw -> MuJoCo wxyz, normalised; ignore a zero quat
            if n > 1e-9:
                self._target_quat = q / n
        if not (msg.type_mask & AttitudeTarget.IGNORE_VELOCITY):
            self._v_cmd = np.array([msg.velocity.x, msg.velocity.y, msg.velocity.z], dtype=float)

    def _try_export_model(self) -> bool:
        """`<model_path の親>/export/` か `models/cruise_policy/export/` を素 torch で読む。

        見つからなければ False を返し、呼び出し元が従来の SB3 経路にフォールバックする。
        """
        from umiusi_rl_control.policy_infer import PolicyRunner

        mp = str(self.get_parameter("model_path").value).strip()
        cands = []
        if mp:
            cands.append(Path(mp).parent / "export")
        cands.append(Path(get_package_share_directory("umiusi_rl_control"))
                     / "models" / "cruise_policy" / "export")
        for d in cands:
            if not (d / "weights.pt").exists():
                continue
            try:
                runner = PolicyRunner(d)
            except Exception as e:  # noqa: BLE001
                self.get_logger().warning(f"export の読み込みに失敗 ({d}): {type(e).__name__}: {e}")
                continue

            class _M:  # model.predict(obs, deterministic=) 互換の薄いラッパ
                def predict(self, obs, deterministic=True):
                    return runner.act(obs, already_normalized=True), None

            self._model = _M()
            self._norm_obs = runner.normalize
            self.get_logger().info(f"policy loaded from {d} (SB3 非依存の素 torch 推論)")
            return True
        return False

    def _ensure_model(self) -> bool:
        if self._model is not None:
            return True
        # 実機優先: SB3 非依存の書き出し (export/) があればそちらを使う。
        # SB3 の policy zip は numpy 2.x で保存されており、ROS Jazzy 標準の numpy 1.26 では
        # `ModuleNotFoundError: numpy._core.numeric` で読めない (custom_objects でもシムでも不可)。
        # export/ 版は torch だけで動き、SB3 と出力が完全一致することを検証済み。
        if self._try_export_model():
            return True
        try:
            from stable_baselines3 import PPO
            from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(
                f"cannot import stable-baselines3 ({type(e).__name__}: {e}); "
                "install it (+ torch, gymnasium) in the ROS runtime environment.",
                throttle_duration_sec=10.0)
            return False
        mp = str(self.get_parameter("model_path").value).strip()
        if not mp:
            mp = str(Path(get_package_share_directory("umiusi_rl_control"))
                     / "models" / "cruise_policy" / "final.zip")
        model_path = Path(mp)
        if not model_path.exists():
            self.get_logger().error(f"model not found: {model_path}", throttle_duration_sec=10.0)
            return False
        self._model = PPO.load(str(model_path), device="cpu")
        stats = model_path.parent / "vecnormalize.pkl"
        if stats.exists():
            dummy = DummyVecEnv([_make_stub_env])
            vn = VecNormalize.load(str(stats), dummy)
            dummy.close()
            rms, clip, eps = vn.obs_rms, vn.clip_obs, vn.epsilon

            def _norm(o):
                return np.clip((o - rms.mean) / np.sqrt(rms.var + eps), -clip, clip).astype(np.float32)
            self._norm_obs = _norm
        else:
            self.get_logger().warning("no vecnormalize.pkl next to the model; using raw obs.")
            self._norm_obs = lambda o: o.astype(np.float32)
        self.get_logger().info(f"policy loaded from {model_path}")
        return True

    def _build_obs(self):
        imu, thr = self._imu, self._thr
        q = imu.orientation
        cur_quat = np.array([q.w, q.x, q.y, q.z], dtype=float)   # ROS xyzw -> MuJoCo wxyz
        g = imu.angular_velocity
        gyro = np.array([g.x, g.y, g.z], dtype=float)
        if self._gyro_to_rad:
            gyro = np.radians(gyro)
        states = [getattr(thr, p) for p in POSITIONS]
        servo_deg = np.array([s.angle for s in states], dtype=float)
        esc_applied = np.array([s.rpm for s in states], dtype=float) / 1000.0

        ori_err = mju_sub_quat(self._target_quat, cur_quat)      # current -> target rot-vec
        servo_n = np.radians(servo_deg) / self._servo_range_rad
        thrust_n = esc_applied
        return np.concatenate([ori_err, gyro, self._v_cmd, servo_n, thrust_n, self._prev_action])

    def _tick(self):
        if not self._arm.armed:            # e-stopped / disarmed: keep asserting the detach
            self._detach_all()
            return
        if self._imu is None or self._thr is None:
            return
        if not self._ensure_model():
            return
        obs = self._build_obs()
        action, _ = self._model.predict(self._norm_obs(obs), deterministic=True)
        action = np.clip(np.asarray(action, dtype=float).reshape(ACT_DIM), -1.0, 1.0)
        if self._publish:
            self._command(action)
        self._prev_action = action

    def _command(self, action):
        for k, p in enumerate(POSITIONS):
            out = ThrusterOutput()
            out.runnable = ThrusterRunnable(esc=True, servo=True)
            out.duty_cycle = float(action[4 + k])
            out.angle = float(action[k]) * self._servo_range_deg   # degrees, matching ros_policy
            self._pubs[p].publish(out)

    def _detach_all(self):
        """DISARM: zero output + runnable false on every thruster -> the control stack detaches
        esc/servo (hardware-level not-allowed). The e-stop / disarm path for the direct loop."""
        if not self._publish:      # compute-only node never commands /cmd, so nothing to detach
            return
        for p in POSITIONS:
            out = ThrusterOutput()
            out.runnable = ThrusterRunnable(esc=False, servo=False)
            out.duty_cycle = 0.0
            out.angle = 0.0
            self._pubs[p].publish(out)

    def stop(self):
        self._detach_all()


def _make_stub_env():
    """Minimal gymnasium.Env with the policy's obs/action spaces — only so VecNormalize.load has a
    venv to bind to (we read its running stats, never step it)."""
    import gymnasium as gym
    from gymnasium import spaces

    class _Stub(gym.Env):
        def __init__(self):
            self.observation_space = spaces.Box(-np.inf, np.inf, (OBS_DIM,), dtype=np.float32)
            self.action_space = spaces.Box(-1.0, 1.0, (ACT_DIM,), dtype=np.float32)

        def reset(self, *, seed=None, options=None):
            return np.zeros(OBS_DIM, dtype=np.float32), {}

        def step(self, action):
            return np.zeros(OBS_DIM, dtype=np.float32), 0.0, False, False, {}

    return _Stub()


def main(args=None):
    rclpy.init(args=args)
    node = RlAttitudeNode()
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
