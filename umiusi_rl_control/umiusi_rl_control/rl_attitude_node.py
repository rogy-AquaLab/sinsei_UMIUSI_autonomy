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

深度モード切替 (``depth_supervisor:=true``, 水圧センサ搭載時のみ): 水圧 (``depth_topic``,
sensor_msgs/FluidPressure) から深度を作り、``target_depth`` との誤差が閾値を超えたら
巡航を一時停止して深度補正に入る — 降下はブレーキ→同梱 3-D ポリシー
(``models/av_cal5_3d_rep103``) の純下バースト、浮上は弱正浮力トリム任せの受動浮上。
状態機械と検証済みパラメータは ``depth_supervisor.py`` 冒頭を参照 (sim リハーサル:
issue #15 のコメント)。**深度モードは max_duty 0.4 が前提** (0.2 では降下できない)。
深度ゼロ点は起動後の最初の水圧サンプル列で取る (水面で起動する前提)。潜った状態で
再ゼロしたいときは ``~/zero_depth`` (std_srvs/Trigger)。診断: ``~/depth`` (Float32, m,
正=深い) と ``~/depth_mode`` (String: horiz/brake/vert/ascend)。
"""

from __future__ import annotations

import math
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import FluidPressure, Imu
from sinsei_umiusi_msgs.msg import ThrusterOutput, ThrusterRunnable
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger

from umiusi_rl_control.arm import ArmState
from umiusi_rl_control.depth_supervisor import HORIZ, VERT, DepthSupervisor
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
DEFAULT_VERT_MODEL = "av_cal5_3d_rep103"  # 深度モードの降下バースト用 (EXPERIMENTAL, 降下専用)
GRAVITY = 9.80665                          # 水圧 -> 深度換算 [m/s^2]
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
        # 実機のサーボ回転センスが ch ごとに反転している場合の補正 (lf, lb, rb, rf)。
        # navigator_node と同じ規約。sim / ポリシーの前提は触らず、実機に出す直前で吸収する。
        self.declare_parameter("servo_sign", [1.0, 1.0, 1.0, 1.0])
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
        # デッドマン: 速度指令が **vel_timeout 秒更新されなかったら 0 に戻す** (0 以下で無効、既定 off)。
        # 狭いプールでの巡航試験向け — teleop が落ちた/操作者が手を離した/`set_attitude --hold` を
        # Ctrl-C した後に、機体が壁まで巡航し続けるのを防ぐ。`--hold` (10 Hz) を使っていれば
        # 通常運転では発動しない。姿勢目標は保持したまま (落とすのは速度だけ)
        self.declare_parameter("vel_timeout", 0.0)
        # --- 深度モード切替 (水圧センサ搭載時のみ。冒頭 docstring と depth_supervisor.py 参照) ---
        self.declare_parameter("depth_supervisor", False)  # true で有効化。**max_duty 0.4 が前提**
        # 起動時専用パラメータは read_only にする (レビュー指摘: 実行中の `ros2 param set` が
        # 黙って無効になるより、はっきり失敗した方がよい)。target_depth だけは実行中変更可
        ro = ParameterDescriptor(read_only=True)
        self.declare_parameter("depth_topic", "/state/pressure", ro)  # sensor_msgs/FluidPressure [Pa]
        self.declare_parameter("target_depth", 0.0)        # 目標深度 [m, 正=深い]。実行中 param set 可
        self.declare_parameter("water_density", 1000.0, ro)  # 淡水 1000 / 海水 ~1025 [kg/m^3]
        self.declare_parameter("vert_model_path", "", ro)  # "" -> 同梱 av_cal5_3d_rep103
        # sim 検証済みの閾値/ゲイン (depth_supervisor.py の既定値)。プールでの追い込み用に
        # 主要 4 つだけ起動時パラメータにする
        self.declare_parameter("depth_d_enter", 0.25, ro)  # 補正に入る誤差 [m]
        self.declare_parameter("depth_d_exit", 0.15, ro)   # 浮上をやめる誤差 [m]
        self.declare_parameter("depth_k", 0.7, ro)         # 降下指令ゲイン [1/s]
        self.declare_parameter("depth_v_vert", 0.2, ro)    # 降下指令上限 [m/s]

        self._hz = float(self.get_parameter("control_hz").value)
        self._dt = 1.0 / self._hz
        self._vel = float(self.get_parameter("vel_cmd").value)
        self._servo_range_deg = float(self.get_parameter("servo_range_deg").value)
        _signs = [float(v) for v in self.get_parameter("servo_sign").value]
        if len(_signs) != len(POSITIONS):
            # 起動時に落とす。誤った符号のまま動かすほうが危険 (ヒーブがロールに化ける)。
            raise ValueError(f"servo_sign needs {len(POSITIONS)} entries {POSITIONS}, got {_signs}")
        self._servo_sign = _signs
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
        self._vel_timeout = float(self.get_parameter("vel_timeout").value)
        self._v_cmd_stamp = None       # 速度指令が最後に更新された時刻 [s] (デッドマン用)
        self._prev_action = np.zeros(ACT_DIM)
        self._model = None
        self._model_error = None       # 構造的な読み込み失敗 (リトライしても直らない)
        self._obs_dim = OBS_DIM        # ポリシー読み込み時に実際の次元で上書きする
        self._imu = None
        self._imu_sanity = ImuSanity(
            max_gyro=float(self.get_parameter("imu_max_gyro").value),
            max_step_deg=float(self.get_parameter("imu_max_step_deg").value),
            enforce=bool(self.get_parameter("imu_sanity_enforce").value))

        # --- 深度モード切替の状態 ---
        self._sup_enabled = bool(self.get_parameter("depth_supervisor").value)
        self._sup = DepthSupervisor(
            d_enter=float(self.get_parameter("depth_d_enter").value),
            d_exit=float(self.get_parameter("depth_d_exit").value),
            k_depth=float(self.get_parameter("depth_k").value),
            v_vert=float(self.get_parameter("depth_v_vert").value))
        self._sup.target_depth = float(self.get_parameter("target_depth").value)
        self._rho = float(self.get_parameter("water_density").value)
        self._vert_model = None            # 3-D ポリシー (深度モード有効時に読み込む)
        self._depth = None                 # 現在深度 [m, 正=深い]。ゼロ点確定まで None
        self._depth_stamp = None           # 最終更新時刻 [s] (鮮度ガード)
        self._p_surface = None             # 水面の圧力 [Pa] (ゼロ点)
        self._p_samples: list[float] = []  # ゼロ点キャプチャ用 (最初の ~0.5 s 分)
        self._last_pub_state = None

        if self._sup_enabled:
            self._sub_depth = self.create_subscription(
                FluidPressure, self.get_parameter("depth_topic").value, self._on_pressure, 5)
            self._pub_depth = self.create_publisher(Float32, "~/depth", 10)
            self._pub_mode = self.create_publisher(String, "~/depth_mode", CURRENT_SETPOINT_QOS)
            self._srv_zero = self.create_service(Trigger, "~/zero_depth", self._on_zero_depth)
            if self._max_duty < 0.35:
                self.get_logger().warning(
                    f"depth_supervisor 有効だが max_duty={self._max_duty:.2f} — sim 実測では "
                    "0.2 で降下不能 (下向き推力が浮力に負ける)。深度試験は 0.4 にすること")

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
        # ポリシーはここで読む (レビュー指摘): torch の import に ~1.5 s (SBC では数秒)
        # かかり、tick 内で読むと武装後の最初の周期で e-stop の処理が止まる窓ができる。
        # spin 前ならまだ何も指令していないので安全
        self._ensure_model()
        self.get_logger().info(
            f"rl_attitude_node: default target=upright v_cmd=[{self._vel:.3f},0,0] m/s @ {self._hz:.0f} Hz "
            f"(publish={self._publish}); live setpoint (AttitudeTarget) on '{sp_topic}'")

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
                self._v_cmd_stamp = self.get_clock().now().nanoseconds * 1e-9
                self._publish_current_setpoint()
                self.get_logger().info(f"vel_cmd={float(p.value):.2f} m/s")
            elif p.name == "vel_timeout":
                self._vel_timeout = float(p.value)
                self.get_logger().info(f"vel_timeout={self._vel_timeout:.1f} s (0 以下で無効)")
            elif p.name == "target_depth":
                self._sup.target_depth = float(p.value)
                self.get_logger().info(f"target_depth={float(p.value):.2f} m")
        return SetParametersResult(successful=True)

    def _on_pressure(self, msg):
        now = self.get_clock().now().nanoseconds * 1e-9
        p = float(msg.fluid_pressure)
        if self._p_surface is None:
            # ゼロ点: 起動後の最初の ~25 サンプルの中央値を水面圧とする (水面で起動する前提)。
            # 潜った状態で起動し直したときは ~/zero_depth で取り直すか、target_depth を
            # 相対値として運用する
            self._p_samples.append(p)
            if len(self._p_samples) >= 25:
                self._p_surface = float(np.median(self._p_samples))
                self.get_logger().info(
                    f"深度ゼロ点を設定: p_surface={self._p_surface:.0f} Pa "
                    f"({len(self._p_samples)} サンプル中央値)")
            return
        self._depth = (p - self._p_surface) / (self._rho * GRAVITY)   # 正 = 深い
        self._depth_stamp = now

    def _on_zero_depth(self, req, res):
        """~/zero_depth (std_srvs/Trigger): いまの圧力を水面 (深度 0) として取り直す。"""
        self._p_surface = None
        self._p_samples = []
        self._depth = None
        res.success = True
        res.message = "深度ゼロ点を再キャプチャ中 (次の ~25 サンプル)"
        return res

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
            self._v_cmd_stamp = self.get_clock().now().nanoseconds * 1e-9
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
            if self._sup_enabled:
                self._load_vert_model()
            return True
        except Exception as e:  # noqa: BLE001
            self._model_error = f"ポリシーを読み込めません: {e}"
            self._model = None       # 片方だけ読めた状態で走らない
            self.get_logger().error(self._model_error)
            return False

    def _load_vert_model(self):
        """深度モードの降下バースト用 3-D ポリシー。水平ポリシーと同じ検証を通す。"""
        mp = str(self.get_parameter("vert_model_path").value).strip()
        d = (Path(mp) if mp else
             Path(get_package_share_directory("umiusi_rl_control")) / "models" / DEFAULT_VERT_MODEL)
        runner = self._load_bundle(d)
        if runner.obs_dim != OBS_DIM:
            raise ValueError(
                f"vert モデルは attitude_velocity (17 次元) が必要です ({d}: {runner.obs_dim})")
        self._vert_model = runner
        self.get_logger().info(f"depth supervisor: vert policy loaded from {d}")

    def _load_model(self, d: Path):
        runner = self._load_bundle(d)
        self._model = runner
        self._obs_dim = runner.obs_dim
        if self._obs_dim == OBS_DIM_NO_VEL:
            self.get_logger().info(
                "attitude タスクのポリシーです (観測に速度指令を含まない)。"
                "vel_cmd / AttitudeTarget.velocity は無視されます")
        self.get_logger().info(f"policy loaded from {d / 'export'} (obs {self._obs_dim}-D, rep103)")

    def _load_bundle(self, d: Path):
        """バンドルを読み、frame 契約・観測次元・golden を検証した PolicyRunner を返す。"""
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
        return runner

    def _build_obs(self, v_cmd, obs_dim):
        imu = self._imu
        cur_quat = np.array(imu.quat, dtype=float)      # ImuSanity が (w,x,y,z) 正規化済みで返す
        gyro = np.array(imu.gyro, dtype=float)          # rad/s, REP-103 のまま (A-2)

        ori_err = mju_sub_quat(self._target_quat, cur_quat)      # current -> target rot-vec
        if not self._hold_yaw:
            # yaw 成分を落とす = その軸まわりの姿勢誤差を 0 として扱う。回転ベクトルの
            # 成分を落とすだけなので特異点が無い (RPY に直すと pitch±90 で破綻する)
            ori_err[YAW_IDX] = 0.0
        parts = [ori_err, gyro]
        if obs_dim == OBS_DIM:                # attitude_velocity のみ速度指令を観測に持つ
            parts.append(v_cmd)
        parts.append(self._prev_action)
        return np.concatenate(parts)

    def _supervise(self):
        """深度モード切替。-> (使うモデル, 観測に入れる v_cmd)。

        prev_action は共有のまま (両ポリシーとも 17 次元で obs レイアウト同一 —
        sim リハーサルで切替の過渡が問題ないことを確認済み)。
        """
        model, v_cmd = self._model, self._v_cmd
        if not (self._sup_enabled and self._vert_model is not None):
            return model, v_cmd
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._depth is None or self._depth_stamp is None or now - self._depth_stamp > 1.0:
            # 深度が来ていない/古い: 補正はしない。鉛直成分だけ落として水平運転を続ける。
            # 状態機械もリセットする (レビュー指摘) — 補正の途中で止まった場合、古い
            # brake/vert タイマーを残すと復帰時に watchdog が誤発火するし、~/depth_mode が
            # 実挙動 (水平フォールバック) と食い違ったまま表示され続ける
            if self._sup.state != HORIZ:
                self._sup.state = HORIZ
            if self._depth_stamp is not None and now - self._depth_stamp > 1.0:
                self.get_logger().warning("深度が 1 s 以上更新されていません — 深度補正を停止中",
                                          throttle_duration_sec=5.0)
            if self._last_pub_state != "stale":
                self._pub_mode.publish(String(data="stale"))
                self._last_pub_state = "stale"
            return model, np.array([v_cmd[0], v_cmd[1], 0.0])
        state, override = self._sup.update(now, self._depth)
        if state != self._last_pub_state:
            self.get_logger().info(
                f"depth mode: {self._last_pub_state or HORIZ} -> {state} "
                f"(depth {self._depth:+.2f} m, target {self._sup.target_depth:+.2f} m, "
                f"retry {self._sup.retries})")
            self._pub_mode.publish(String(data=state))
            self._last_pub_state = state
        self._pub_depth.publish(Float32(data=float(self._depth)))
        if override is None:
            # 水平モード: 斜め指令を作らない (鉛直成分は必ず 0 に丸める — sim 実測で
            # 斜めはどのポリシーも不安定)
            return model, np.array([v_cmd[0], v_cmd[1], 0.0])
        return (self._vert_model if state == VERT else model), override

    def _tick(self):
        if not self._arm.armed:            # e-stopped / disarmed: keep asserting the detach
            self._detach_all()
            # 補正の途中で disarm されたら状態機械も仕切り直す (古いブレーキタイマー等を残さない)
            if self._sup.state != HORIZ:
                self._sup.state = HORIZ
                self._last_pub_state = None
            return
        if self._imu is None:
            return
        if not self._ensure_model():
            return
        # デッドマン: 速度指令が更新されないまま vel_timeout を過ぎたら 0 に戻す (姿勢保持は継続)
        if self._vel_timeout > 0.0 and np.any(self._v_cmd != 0.0):
            now = self.get_clock().now().nanoseconds * 1e-9
            if self._v_cmd_stamp is None:      # launch 指定の vel_cmd も対象 (初回 tick 起点)
                self._v_cmd_stamp = now
            if now - self._v_cmd_stamp > self._vel_timeout:
                self._v_cmd = np.zeros(3)
                self._v_cmd_stamp = None
                self._publish_current_setpoint()
                self.get_logger().warning(
                    f"速度指令が {self._vel_timeout:.1f} s 更新されなかったので 0 に戻しました (デッドマン)")
        model, v_cmd = self._supervise()
        # 鉛直指令インターロック (レビュー指摘): 鉛直速度が学習分布内のポリシー
        # (export/meta.json の vertical_ok, 3-D vectoring 系) 以外には z 成分を渡さない。
        # 水平専用ポリシーに鉛直指令が入ると姿勢が崩壊する (sim 実測 75〜122°)。
        # set_attitude --vel 0 0 -0.2 を間違ったポリシーで打っても機体は水平ホールドを続ける
        if v_cmd[2] != 0.0 and not model.meta.get("vertical_ok", False):
            self.get_logger().warning(
                "鉛直速度指令を 0 にクランプしました — このポリシーは水平専用です "
                "(vertical_ok なし)。降下バーストは av_cal5_3d_rep103 で",
                throttle_duration_sec=5.0)
            v_cmd = np.array([v_cmd[0], v_cmd[1], 0.0])
        action = model.act(self._build_obs(v_cmd, model.obs_dim))
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
            # degrees, matching the plugin。ch 別のサーボ符号は **ここ** (実機に出す境界) でだけ
            # 当てる — _servo_cmd は sim 規約のまま保つ。範囲外は CAN フレームが送れずに落ちる
            # ので、ハードの ±90 に収めてから出す。
            out.angle = max(-90.0, min(90.0, float(self._servo_cmd[k]) * self._servo_sign[k]))
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
