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

Command: holds UPRIGHT (target = identity) + cruises forward (body +X at ``vel_cmd`` m/s, **既定 0**),
and can be overridden in REAL TIME by publishing an ``umiusi_rl_control_msgs/AttitudeTarget`` on
``~/current_setpoint`` に**いま適用されている目標値**を latch で出す (診断用):
``ros2 topic echo --once /rl_attitude_node/current_setpoint``。

``setpoint_topic`` (target attitude quaternion + feed-forward velocity in the target-body frame;
``type_mask`` selects which fields apply). Last message wins; a teleop / joystick controller just
publishes it.

SAFETY: ``~/estop`` (std_msgs/Bool, true) or ``~/arm`` (std_srvs/SetBool, data:false) DISARMs
immediately — the loop stops predicting and asserts a DETACH every tick (ThrusterOutput runnable
esc/servo = false, zero output), so the control stack releases the thrusters. Re-arm with the
``~/arm`` service (data:true)。**既定は disarmed で起動する** (``start_armed:=true`` で
起動と同時に武装)。あわせて ``vel_cmd`` の既定も 0 なので、**武装しても勝手には前進しない**。

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
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Imu
from sinsei_umiusi_msgs.msg import ThrusterOutput, ThrusterRunnable, ThrusterStateAll

from umiusi_rl_control.arm import ArmState
from umiusi_rl_control.imu_sanity import ImuSanity
from umiusi_rl_control_msgs.msg import AttitudeTarget

POSITIONS = ("lf", "lb", "rb", "rf")
CMD_PREFIX = "/cmd/direct/thruster_controller/output_"
OBS_DIM = 25
ACT_DIM = 8


# 現在の目標値は latch する。**後から `ros2 topic echo` しても最新値が読める**ように
# するため (VOLATILE だと、実行中に繋いでも次の更新まで何も出ない)。
CURRENT_SETPOINT_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def _quat_to_rpy_deg(q):
    """(w, x, y, z) -> roll/pitch/yaw [deg] (ZYX)。ログ表示用。"""
    w, x, y, z = (float(v) for v in q)
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    sinp = 2.0 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2.0, sinp) if abs(sinp) >= 1.0 else math.asin(sinp)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return tuple(math.degrees(v) for v in (roll, pitch, yaw))


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
        # **既定は 0**。巡航ポリシー自体は 0.4 m/s の前進込みで学習しているが、既定を 0.4 に
        # すると「起動しただけで前進指令が出る」ことになる。前進させたいときは vel_cmd で
        # 明示するか、実行中に AttitudeTarget の velocity で与える。
        self.declare_parameter("vel_cmd", 0.0)             # forward (+X) commanded speed [m/s]
        self.declare_parameter("servo_range_deg", 90.0)
        self.declare_parameter("imu_max_gyro", 10.0)       # IMU サニティ: 角速度上限 [rad/s]
        self.declare_parameter("imu_max_step_deg", 30.0)   # IMU サニティ: 姿勢跳躍上限 [deg]
        # 既定は「検出するが破棄しない」。実機では該当が 0.44% しかないうえ、フィルタ自身の
        # 誤爆 (姿勢基準が飛ぶと復帰できない) のほうが被害が大きかった。閾値を決めるための
        # データが貯まるまでは観測に徹する。true にすると従来どおり破棄する。
        self.declare_parameter("imu_sanity_enforce", False)
        self.declare_parameter("gyro_deg_per_sec", False)  # convert IMU gyro deg/s -> rad/s (see caveats)
        self.declare_parameter("publish", True)            # False = predict only, do not command
        # 姿勢保持のうち **yaw だけを切れる**。実験中に機体を手で回すと、yaw の目標が
        # 起動時のままなので戻そうとして回り続ける (実機で踏んだ)。水中では磁気の影響や
        # ドリフトもあるので、roll/pitch だけ保ちたい場面が多い。
        # 実行中に `ros2 param set /rl_attitude_node hold_yaw false` で切り替えられる。
        self.declare_parameter("hold_yaw", True)
        self.declare_parameter("yaw_axis", "y")            # 姿勢誤差のどの成分が yaw か (x/y/z)
        # duty_cycle の絶対値上限。1.0 = 制限なし。**まず小さい値で試すためのもの**
        self.declare_parameter("max_duty", 1.0)
        # **既定は disarmed**。起動と同時にスラスタへ指令が出るのを避ける。
        # `~/arm` サービス (data:true) で武装してから動かす。
        self.declare_parameter("start_armed", False)       # True = 起動と同時に武装する
        # Real-time setpoint (hold last; until a message arrives, use the launch defaults below).
        self.declare_parameter("setpoint_topic", "~/setpoint")   # umiusi_rl_control_msgs/AttitudeTarget

        self._hz = float(self.get_parameter("control_hz").value)
        self._dt = 1.0 / self._hz
        self._vel = float(self.get_parameter("vel_cmd").value)
        self._servo_range_deg = float(self.get_parameter("servo_range_deg").value)
        self._servo_range_rad = math.radians(self._servo_range_deg)
        self._gyro_to_rad = bool(self.get_parameter("gyro_deg_per_sec").value)
        self._publish = bool(self.get_parameter("publish").value)
        self._hold_yaw = bool(self.get_parameter("hold_yaw").value)
        self._max_duty = abs(float(self.get_parameter("max_duty").value))
        axis = str(self.get_parameter("yaw_axis").value).lower()
        if axis not in ("x", "y", "z"):
            raise ValueError(f"yaw_axis は x/y/z のいずれか (指定: {axis!r})")
        self._yaw_idx = {"x": 0, "y": 1, "z": 2}[axis]

        self._target_quat = np.array([1.0, 0.0, 0.0, 0.0])   # identity = hold upright/level
        self._v_cmd = np.array([self._vel, 0.0, 0.0])         # cruise along body +X
        self._prev_action = np.zeros(ACT_DIM)
        self._model = None
        self._norm_obs = None
        self._imu = None
        self._imu_sanity = ImuSanity(
            max_gyro=float(self.get_parameter("imu_max_gyro").value),
            max_step_deg=float(self.get_parameter("imu_max_step_deg").value),
            enforce=bool(self.get_parameter("imu_sanity_enforce").value))
        self._thr = None

        self._sub_imu = self.create_subscription(
            Imu, self.get_parameter("imu_topic").value, self._on_imu, 1)
        self._sub_thr = self.create_subscription(
            ThrusterStateAll, self.get_parameter("thruster_state_topic").value, self._on_thr, 1)
        # Real-time setpoint (optional): last message wins; absence keeps the defaults above.
        sp_topic = self.get_parameter("setpoint_topic").value
        self._sub_sp = self.create_subscription(AttitudeTarget, sp_topic, self._on_setpoint, 1)
        self._pubs = {p: self.create_publisher(ThrusterOutput, CMD_PREFIX + p, 10) for p in POSITIONS}
        # いま何を目標にして動いているかを外から確認できるようにする。setpoint (購読側) は
        # 「送る」ためのもので、type_mask で一部だけ更新されることもあるため、
        # **実際に適用されている値**をこちらに出す
        self._pub_current_sp = self.create_publisher(
            AttitudeTarget, "~/current_setpoint", CURRENT_SETPOINT_QOS)
        self._arm = ArmState(self, self._detach_all,
                             start_armed=bool(self.get_parameter("start_armed").value))
        self._timer = self.create_timer(self._dt, self._tick)
        self.add_on_set_parameters_callback(self._on_set_params)
        self._publish_current_setpoint()
        self.get_logger().info(
            f"rl_attitude_node: default target=upright v_cmd=[{self._vel:.3f},0,0] m/s @ {self._hz:.0f} Hz "
            f"(publish={self._publish}); live setpoint (AttitudeTarget) on '{sp_topic}'; "
            "loading policy on first state...")

    def _on_set_params(self, params):
        """`ros2 param set` を実行中に効かせる (hold_yaw / max_duty / vel_cmd)。"""
        for p in params:
            if p.name == "hold_yaw":
                self._hold_yaw = bool(p.value)
                self.get_logger().info(
                    f"hold_yaw={self._hold_yaw}"
                    f"{'' if self._hold_yaw else ' — yaw は保持しません (roll/pitch のみ)'}")
            elif p.name == "max_duty":
                self._max_duty = abs(float(p.value))
                self.get_logger().info(f"max_duty={self._max_duty:.2f}")
            elif p.name == "vel_cmd":
                self._v_cmd = np.array([float(p.value), 0.0, 0.0])
                self._publish_current_setpoint()
                self.get_logger().info(f"vel_cmd={float(p.value):.2f} m/s")
        return SetParametersResult(successful=True)

    def _on_imu(self, msg):
        # 実機の BNO055 は化けサンプルを混ぜてくる。姿勢と角速度を直接ポリシーの観測に
        # 入れるので、1 発のスパイクで指令が跳ねる。ただし **既定では検出するだけで弾かない**
        # (`imu_sanity_enforce`)。理由は imu_sanity.py 冒頭。
        q, g = msg.orientation, msg.angular_velocity
        resyncs = self._imu_sanity.resyncs
        sample, reason = self._imu_sanity.update((q.w, q.x, q.y, q.z), (g.x, g.y, g.z))
        if reason is not None:
            self.get_logger().warning(self._imu_sanity.describe(reason),
                                      throttle_duration_sec=5.0)
        # enforce=False では reason が付いたまま採用されるので、独立した if で見る
        if self._imu_sanity.resyncs > resyncs:
            # 姿勢基準そのものが飛んだ。フィルタは再同期したが、目標姿勢は飛ぶ前の基準で
            # 与えられているので、**目標を入れ直す必要がある**。黙って進むと危険。
            self.get_logger().warning(
                f"IMU の姿勢基準が飛んだので再同期しました "
                f"(通算 {self._imu_sanity.resyncs} 回)。目標姿勢を与え直してください")
        self._imu = sample

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
        self._publish_current_setpoint()
        r, p, y = _quat_to_rpy_deg(self._target_quat)
        self.get_logger().info(
            f"目標を更新: roll={r:+.1f} pitch={p:+.1f} yaw={y:+.1f} deg"
            f"  速度=[{self._v_cmd[0]:.2f},{self._v_cmd[1]:.2f},{self._v_cmd[2]:.2f}] m/s")

    def _publish_current_setpoint(self):
        """いま適用されている目標値を `~/current_setpoint` に出す (latch)。

            ros2 topic echo --once /rl_attitude_node/current_setpoint
        """
        msg = AttitudeTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        w, x, y, z = (float(v) for v in self._target_quat)
        msg.orientation.w, msg.orientation.x = w, x
        msg.orientation.y, msg.orientation.z = y, z
        msg.velocity.x = float(self._v_cmd[0])
        msg.velocity.y = float(self._v_cmd[1])
        msg.velocity.z = float(self._v_cmd[2])
        msg.type_mask = 0        # 現在値なので「両方が有効」
        self._pub_current_sp.publish(msg)

    def _try_export_model(self) -> bool:
        """素 torch で export ディレクトリを読む。

        `model_path` を指定したときは `<model_path の親>/export/` **だけ**、
        未指定のときは `models/cruise_policy/export/` だけを見る (C-1)。
        見つからなければ False を返し、呼び出し元が従来の SB3 経路にフォールバックする。
        """
        try:
            from umiusi_rl_control.policy_infer import PolicyRunner
        except Exception as e:  # noqa: BLE001  (torch 未導入など)
            self.get_logger().error(
                f"素 torch 推論を読み込めません ({type(e).__name__}: {e}); "
                "torch を入れるか SB3 経路を使ってください", throttle_duration_sec=10.0)
            return False

        mp = str(self.get_parameter("model_path").value).strip()
        if mp:
            # model_path を明示したときは **その隣の export/ だけ** を見る。
            # ここで package share のバンドル済みポリシーに落ちると、新しいポリシーを
            # 試しているつもりで巡航ポリシーが動く事故になる。
            cands = [Path(mp).parent / "export"]
        else:
            cands = [Path(get_package_share_directory("umiusi_rl_control"))
                     / "models" / "cruise_policy" / "export"]
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
        cur_quat = np.array(imu.quat, dtype=float)      # ImuSanity が (w,x,y,z) 正規化済みで返す
        gyro = np.array(imu.gyro, dtype=float)
        if self._gyro_to_rad:
            gyro = np.radians(gyro)
        states = [getattr(thr, p) for p in POSITIONS]
        servo_deg = np.array([s.angle for s in states], dtype=float)
        esc_applied = np.array([s.rpm for s in states], dtype=float) / 1000.0

        ori_err = mju_sub_quat(self._target_quat, cur_quat)      # current -> target rot-vec
        if not self._hold_yaw:
            # yaw 成分を落とす = その軸まわりの姿勢誤差を 0 として扱う。回転ベクトルの
            # 成分を落とすだけなので特異点が無い (RPY に直すと pitch±90 で破綻する)
            ori_err[self._yaw_idx] = 0.0
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
            out.duty_cycle = float(np.clip(action[4 + k], -self._max_duty, self._max_duty))
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
