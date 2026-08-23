"""rl_attitude_node — drive the thrusters with a trained RL attitude(-velocity) policy.

SB3/mujoco 非依存: 同梱バンドルの ``export/`` (weights.pt + obs_norm.npz + meta.json) を
素 torch で推論する (``policy_infer.PolicyRunner``)。ループ:

  * SUBSCRIBE  /state/imu (sensor_msgs/Imu)
  * ポリシーの観測を組み立てる — layout [ori_err(3), gyro(3), v_cmd(3), prev_action(8)] の
    17 次元 (attitude_velocity) / v_cmd を除いた 14 次元 (attitude)。
    **サーボ角・推力は観測に入れない** (実機の /state/thruster_state_all は指令のエコーで、
    正帰還に入る — known_issues A-11)。prev_action は自分が出した action をそのまま使う
    (sim 側 ``proprio_mode: action`` と同じ)。
  * PUBLISH the four direct-override /cmd/direct/thruster_controller/output_{lf,lb,rb,rf}
    (ThrusterOutput, runnable=true), action [servo x4, esc x4] -> {angle, duty_cycle}.

FRAME 契約: ポリシーは **REP-103 body-frame (x前/y左/z上) の観測を消費する**
(``export/meta.json`` の ``obs_frame: rep103`` を起動時に検証する)。IMU の quat/gyro は
**軸変換せずそのまま**観測に入れる。前提は「IMU が REP-103 で publish していること」 —
実験前のドライ確認 (issue #15 A-4) でずれていたら **IMU ドライバ側** (AXIS_MAP) を直す。

配備前検証: バンドルに ``golden.npz`` (sim で記録した観測→行動ペア) があれば、読み込み時に
全ベクトルを再生して一致確認する (issue #15 A-5)。**不一致ならポリシーを動かさない** —
コピー・正規化統計・観測レイアウト・frame のどれかが壊れている。

Command: holds UPRIGHT (target = identity)。前進は **既定 0** (新ポリシーは停止保持も学習
分布内)。目標は ``umiusi_rl_control_msgs/AttitudeTarget`` を ``setpoint_topic`` に publish
して REAL TIME に上書きできる (type_mask で姿勢/速度を選択、last wins)。
いま適用されている目標値は ``~/current_setpoint`` に latch で出す (診断用):
``ros2 topic echo --once /rl_attitude_node/current_setpoint``。

SAFETY: ``~/estop`` (std_msgs/Bool, true) or ``~/arm`` (std_srvs/SetBool, data:false) DISARMs
immediately — the loop stops predicting and asserts a DETACH every tick (ThrusterOutput runnable
esc/servo = false, zero output), so the control stack releases the thrusters. Re-arm with the
``~/arm`` service (data:true)。**既定は disarmed で起動する** (``start_armed:=true`` で
起動と同時に武装)。
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
from sinsei_umiusi_msgs.msg import ThrusterOutput, ThrusterRunnable

from umiusi_rl_control.arm import ArmState
from umiusi_rl_control.imu_sanity import ImuSanity
from umiusi_rl_control.thruster_limits import slew
from umiusi_rl_control_msgs.msg import AttitudeTarget

POSITIONS = ("lf", "lb", "rb", "rf")
CMD_PREFIX = "/cmd/direct/thruster_controller/output_"
# 観測の次元は **ポリシーのタスクによって変わる**:
#   attitude_velocity (巡航) = 17 … [ori_err 3, gyro 3, v_cmd 3, prev_action 8]
#   attitude          (姿勢のみ) = 14 … 上から v_cmd を除いたもの
# 読み込んだポリシーの入力次元から判断して、観測の組み立てを合わせる。
OBS_DIM = 17
OBS_DIM_NO_VEL = 14
ACT_DIM = 8
DEFAULT_MODEL = "av_cal1_best_rep103"     # 本命 (issue #15 B 表)。同梱 models/ から選ぶ
# REP-103 (x前/y左/z上) では yaw は z 軸まわり。姿勢誤差 rot-vec の z 成分を落とすと
# yaw 保持だけ切れる (hold_yaw=false)
YAW_IDX = 2


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
        # "" -> 同梱の models/av_cal1_best_rep103。ディレクトリ (export/ を含む) を指す
        self.declare_parameter("model_path", "")
        self.declare_parameter("imu_topic", "/state/imu")
        self.declare_parameter("control_hz", 50.0)
        # 前進速度の既定は 0。新ポリシーは停止保持 (v_cmd=0) も学習分布内なので、
        # 旧 A-9 (0 が分布外で飽和する) は当てはまらない。武装しても勝手に前進しない。
        self.declare_parameter("vel_cmd", 0.0)             # forward (+X) commanded speed [m/s]
        self.declare_parameter("servo_range_deg", 90.0)
        self.declare_parameter("imu_max_gyro", 10.0)       # IMU サニティ: 角速度上限 [rad/s]
        self.declare_parameter("imu_max_step_deg", 30.0)   # IMU サニティ: 姿勢跳躍上限 [deg]
        # 既定は「検出するが破棄しない」。実機では該当が 0.44% しかないうえ、フィルタ自身の
        # 誤爆 (姿勢基準が飛ぶと復帰できない) のほうが被害が大きかった。閾値を決めるための
        # データが貯まるまでは観測に徹する。true にすると従来どおり破棄する。
        self.declare_parameter("imu_sanity_enforce", False)
        self.declare_parameter("publish", True)            # False = predict only, do not command
        # 姿勢保持のうち **yaw だけを切れる**。実験中に機体を手で回すと、yaw の目標が
        # 起動時のままなので戻そうとして回り続ける (実機で踏んだ)。水中では磁気の影響や
        # ドリフトもあるので、roll/pitch だけ保ちたい場面が多い。
        # 実行中に `ros2 param set /rl_attitude_node hold_yaw false` で切り替えられる。
        self.declare_parameter("hold_yaw", True)
        # duty_cycle の絶対値上限。1.0 = 制限なし。**既定は 0.2** — sim の事前評価で
        # 姿勢・巡航とも最良だった値 (issue #15 A-3)。問題なければ 0.4 へ上げる。
        self.declare_parameter("max_duty", 0.2)
        # **指令のレート制限。sim と同じ値を既定にする** (configs/umiusi.yaml の
        # servo_slew_deg_per_s / thrust_slew_per_s)。sim はポリシーの指令をこれで
        # 平滑化してから物理に入れており、実機側に無いと sim2real ギャップになる。
        # 新ポリシーは servo slew 100–500 deg/s の DR で学習済み。0 以下で無効。
        self.declare_parameter("servo_slew_deg_per_s", 250.0)
        self.declare_parameter("thrust_slew_per_s", 4.0)
        # **既定は disarmed**。起動と同時にスラスタへ指令が出るのを避ける。
        # `~/arm` サービス (data:true) で武装してから動かす。
        self.declare_parameter("start_armed", False)       # True = 起動と同時に武装する
        # Real-time setpoint (hold last; until a message arrives, use the launch defaults below).
        self.declare_parameter("setpoint_topic", "~/setpoint")   # umiusi_rl_control_msgs/AttitudeTarget

        self._hz = float(self.get_parameter("control_hz").value)
        self._dt = 1.0 / self._hz
        self._vel = float(self.get_parameter("vel_cmd").value)
        self._servo_range_deg = float(self.get_parameter("servo_range_deg").value)
        self._publish = bool(self.get_parameter("publish").value)
        self._hold_yaw = bool(self.get_parameter("hold_yaw").value)
        self._max_duty = abs(float(self.get_parameter("max_duty").value))
        self._servo_slew = float(self.get_parameter("servo_slew_deg_per_s").value)
        self._thrust_slew = float(self.get_parameter("thrust_slew_per_s").value)
        # レート制限を掛けた「いま出している指令」。sim の servo_ctrl / esc_current に相当
        self._servo_cmd = np.zeros(len(POSITIONS))
        self._duty_cmd = np.zeros(len(POSITIONS))

        self._target_quat = np.array([1.0, 0.0, 0.0, 0.0])   # identity = hold upright/level
        self._v_cmd = np.array([self._vel, 0.0, 0.0])         # cruise along body +X
        self._prev_action = np.zeros(ACT_DIM)
        self._model = None
        self._model_error = None       # 構造的な読み込み失敗 (リトライしても直らない)
        self._obs_dim = OBS_DIM        # ポリシー読み込み時に実際の次元で上書きする
        self._imu = None
        self._imu_sanity = ImuSanity(
            max_gyro=float(self.get_parameter("imu_max_gyro").value),
            max_step_deg=float(self.get_parameter("imu_max_step_deg").value),
            enforce=bool(self.get_parameter("imu_sanity_enforce").value))

        self._sub_imu = self.create_subscription(
            Imu, self.get_parameter("imu_topic").value, self._on_imu, 1)
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
        """`ros2 param set` を実行中に効かせる (hold_yaw / max_duty / vel_cmd / slew)。"""
        for p in params:
            if p.name == "hold_yaw":
                self._hold_yaw = bool(p.value)
                self.get_logger().info(
                    f"hold_yaw={self._hold_yaw}"
                    f"{'' if self._hold_yaw else ' — yaw は保持しません (roll/pitch のみ)'}")
            elif p.name == "max_duty":
                self._max_duty = abs(float(p.value))
                self.get_logger().info(f"max_duty={self._max_duty:.2f}")
            elif p.name == "servo_slew_deg_per_s":
                self._servo_slew = float(p.value)
                self.get_logger().info(f"servo_slew={self._servo_slew:.1f} deg/s")
            elif p.name == "thrust_slew_per_s":
                self._thrust_slew = float(p.value)
                self.get_logger().info(f"thrust_slew={self._thrust_slew:.2f} /s")
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

    def _on_setpoint(self, msg):
        # umiusi_rl_control_msgs/AttitudeTarget. type_mask selects which fields to apply (mavros-style):
        # a masked-out field keeps its previous value; default mask 0 = update both.
        if not (msg.type_mask & AttitudeTarget.IGNORE_ATTITUDE):
            q = np.array([msg.orientation.w, msg.orientation.x, msg.orientation.y, msg.orientation.z],
                         dtype=float)
            n = np.linalg.norm(q)          # normalised; ignore a zero quat
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

    def _model_dir(self) -> Path:
        mp = str(self.get_parameter("model_path").value).strip()
        if mp:
            p = Path(mp)
            return p if p.is_dir() else p.parent     # final.zip を指されてもディレクトリに直す
        return (Path(get_package_share_directory("umiusi_rl_control"))
                / "models" / DEFAULT_MODEL)

    def _ensure_model(self) -> bool:
        """バンドルの export/ を素 torch で読み、frame 契約と golden を検証してから使う。

        構造的な失敗 (export が無い / frame 不一致 / golden 不一致 / 未対応次元) は
        リトライしても直らないので、一度だけエラーを出して以後は動かさない。
        """
        if self._model is not None:
            return True
        if self._model_error is not None:
            self.get_logger().error(self._model_error, throttle_duration_sec=30.0)
            return False
        try:
            self._load_model(self._model_dir())
            return True
        except Exception as e:  # noqa: BLE001
            self._model_error = f"ポリシーを読み込めません: {e}"
            self.get_logger().error(self._model_error)
            return False

    def _load_model(self, d: Path):
        from umiusi_rl_control.policy_infer import PolicyRunner   # torch は遅延 import

        export = d / "export"
        if not (export / "weights.pt").exists():
            raise FileNotFoundError(
                f"{export}/weights.pt がありません。バンドルは export/ (weights.pt + obs_norm.npz "
                "+ meta.json) を含むディレクトリを指定してください "
                "(SB3 の final.zip 単体は実機の numpy では読めません)")
        runner = PolicyRunner(export)

        # frame 契約 (issue #15 A-2): rep103 の観測を消費するポリシーだけを許す
        frame = runner.meta.get("obs_frame", "unknown")
        if frame != "rep103":
            raise ValueError(
                f"obs_frame={frame!r} のポリシーです ({export}/meta.json)。このノードは IMU を "
                "無変換 (REP-103) で観測に入れるので、rep103 変換済みバンドルだけを使えます")
        if runner.obs_dim not in (OBS_DIM, OBS_DIM_NO_VEL):
            raise ValueError(
                f"対応していない観測次元 {runner.obs_dim} です ({export})。"
                f"{OBS_DIM} (attitude_velocity) か {OBS_DIM_NO_VEL} (attitude) のみ対応します "
                "(servo/thrust を観測に含む旧 25/22 次元ポリシーは廃止 — A-11 のエコー問題)")

        # 配備前検証 (issue #15 A-5): sim で記録した golden vectors を実機の推論経路で再生
        golden = d / "golden.npz"
        if golden.exists():
            g = np.load(golden)
            worst = max(float(np.abs(runner.act(o) - a).max())
                        for o, a in zip(g["obs"], g["act"]))
            if worst > 1e-4:
                raise ValueError(
                    f"golden 検証 FAIL: max|action-golden|={worst:.2e} ({golden})。"
                    "重み・正規化統計・観測レイアウトのどれかが sim と食い違っています")
            self.get_logger().info(
                f"golden 検証 PASS: {len(g['obs'])} vectors, max err {worst:.1e}")
        else:
            self.get_logger().warning(f"{golden} が無いので配備前検証をスキップします")

        self._model = runner
        self._obs_dim = runner.obs_dim
        if self._obs_dim == OBS_DIM_NO_VEL:
            self.get_logger().info(
                "attitude タスクのポリシーです (観測に速度指令を含まない)。"
                "vel_cmd / AttitudeTarget.velocity は無視されます")
        self.get_logger().info(f"policy loaded from {export} (obs {self._obs_dim}-D, rep103)")

    def _build_obs(self):
        imu = self._imu
        cur_quat = np.array(imu.quat, dtype=float)      # ImuSanity が (w,x,y,z) 正規化済みで返す
        gyro = np.array(imu.gyro, dtype=float)          # rad/s, REP-103 のまま (A-2)

        ori_err = mju_sub_quat(self._target_quat, cur_quat)      # current -> target rot-vec
        if not self._hold_yaw:
            # yaw 成分を落とす = その軸まわりの姿勢誤差を 0 として扱う。回転ベクトルの
            # 成分を落とすだけなので特異点が無い (RPY に直すと pitch±90 で破綻する)
            ori_err[YAW_IDX] = 0.0
        parts = [ori_err, gyro]
        if self._obs_dim == OBS_DIM:          # attitude_velocity のみ速度指令を観測に持つ
            parts.append(self._v_cmd)
        parts.append(self._prev_action)
        return np.concatenate(parts)

    def _tick(self):
        if not self._arm.armed:            # e-stopped / disarmed: keep asserting the detach
            self._detach_all()
            return
        if self._imu is None:
            return
        if not self._ensure_model():
            return
        action = self._model.act(self._build_obs())
        action = np.clip(np.asarray(action, dtype=float).reshape(ACT_DIM), -1.0, 1.0)
        if self._publish:
            self._command(action)
        self._prev_action = action

    def _command(self, action):
        # sim と同じレート制限を通してから出す。ポリシーは毎ステップ飽和した指令を出しうるが、
        # sim ではここで平滑化されたものが物理に入り、観測にも返っていた
        servo_target = np.asarray(action[:4], dtype=float) * self._servo_range_deg
        duty_target = np.clip(np.asarray(action[4:], dtype=float), -self._max_duty, self._max_duty)
        self._servo_cmd = slew(self._servo_cmd, servo_target, self._servo_slew, self._dt)
        self._duty_cmd = slew(self._duty_cmd, duty_target, self._thrust_slew, self._dt)
        for k, p in enumerate(POSITIONS):
            out = ThrusterOutput()
            out.runnable = ThrusterRunnable(esc=True, servo=True)
            out.duty_cycle = float(self._duty_cmd[k])
            out.angle = float(self._servo_cmd[k])                  # degrees, matching the plugin
            self._pubs[p].publish(out)

    def _detach_all(self):
        """DISARM: zero output + runnable false on every thruster -> the control stack detaches
        esc/servo (hardware-level not-allowed). The e-stop / disarm path for the direct loop."""
        if not self._publish:      # compute-only node never commands /cmd, so nothing to detach
            return
        # 停止はレート制限を通さない (安全側。次に武装したとき 0 から積み直す)
        self._servo_cmd[:] = 0.0
        self._duty_cmd[:] = 0.0
        for p in POSITIONS:
            out = ThrusterOutput()
            out.runnable = ThrusterRunnable(esc=False, servo=False)
            out.duty_cycle = 0.0
            out.angle = 0.0
            self._pubs[p].publish(out)

    def stop(self):
        self._detach_all()


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
