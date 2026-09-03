"""rl_attitude_node — 学習済み RL 方策でスラスタを駆動する。

/state/imu を購読 -> 観測を組み立てて素 torch で推論 (SB3/mujoco 非依存) ->
/cmd/direct/thruster_controller/output_{lf,lb,rb,rf} へ publish。

観測レイアウト、action_mode、golden.npz と obs_fields の役割分担、深度モード、
起動方法は README を参照。

実装上の注意:
  * サーボ角・推力を観測に入れない。/state/thruster_state_all は指令のエコーで
    正帰還に入る (known_issues A-11)
  * IMU の quat/gyro は軸変換せずそのまま入れる。ずれていたら IMU ドライバ側
    (AXIS_MAP) を直す (known_issues A-13)
  * 既定は disarmed。disarm 中は毎 tick DETACH を assert し続ける (1 回では
    取りこぼす)
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

from umiusi_common.arm import ArmState
from umiusi_rl_control.depth_supervisor import HORIZ, VERT, DepthSupervisor
from umiusi_common.imu_sanity import ImuSanity
from umiusi_rl_control.mode_action import MODE_DIM, ModeAction
from umiusi_rl_control.thruster_limits import slew
from umiusi_rl_control_msgs.msg import AttitudeTarget

POSITIONS = ("lf", "lb", "rb", "rf")
CMD_PREFIX = "/cmd/direct/thruster_controller/output_"
# 観測の次元でタスクを判別する。並びと max_duty 末尾の契約は README「観測レイアウト」
ACT_DIM = 8
ACTION_MODE_DIRECT = "direct"
ACTION_MODE_MODES = "modes"
OBS_DIM_CAP = 18
OBS_DIM = 17
OBS_DIM_NO_VEL = 14
# 速度指令を観測に持つ次元 (attitude タスクだけが持たない)
OBS_DIMS_WITH_VEL = (OBS_DIM_CAP, OBS_DIM)
OBS_DIMS_SUPPORTED = (OBS_DIM_CAP, OBS_DIM, OBS_DIM_NO_VEL)
# 18 次元ポリシーの学習時 duty 上限の分布。観測に入れる値だけここへクランプする
# (実際の duty クリップはオペレータの max_duty のまま)。理由は README
MAX_DUTY_OBS_RANGE = (0.2, 0.4)
OBS_FIELDS = {
    OBS_DIM_CAP: (("ori_err", 3), ("gyro", 3), ("v_cmd", 3), ("prev_action", ACT_DIM),
                  ("max_duty", 1)),
    OBS_DIM: (("ori_err", 3), ("gyro", 3), ("v_cmd", 3), ("prev_action", ACT_DIM)),
    OBS_DIM_NO_VEL: (("ori_err", 3), ("gyro", 3), ("prev_action", ACT_DIM)),
}
DEFAULT_MODEL = "av_cal1_best_rep103"     # 本命 (issue #15 B 表)。同梱 models/ から選ぶ
DEFAULT_VERT_MODEL = "av_cal5_3d_rep103"  # 深度モードの降下バースト用 (EXPERIMENTAL, 降下専用)
GRAVITY = 9.80665                          # 水圧 -> 深度換算 [m/s^2]
# REP-103 (x前/y左/z上) では yaw は z 軸まわり。姿勢誤差 rot-vec の z 成分を落とすと
# yaw 保持だけ切れる (hold_yaw=false)
YAW_IDX = 2


# 現在の目標値は latch する。後から ros2 topic echo しても最新値が読めるように
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
    """Rotation 3-vector taking qb to qa — a numpy reimplementation of MuJoCo's
    mju_subQuat (verified bit-identical), so we avoid a mujoco dependency on the robot."""
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
        # 旧 A-9 (0 が分布外で飽和する) は当てはまらない。arm しても勝手に前進しない。
        self.declare_parameter("vel_cmd", 0.0)             # forward (+X) commanded speed [m/s]
        self.declare_parameter("servo_range_deg", 90.0)
        # 実機のサーボ回転センスが ch ごとに反転している場合の補正 (lf, lb, rb, rf)。
        # navigator_node と同じ規約。sim / ポリシーの前提は触らず、実機に出す直前で吸収する。
        self.declare_parameter("servo_sign", [1.0, 1.0, 1.0, 1.0])
        self.declare_parameter("imu_max_gyro", 10.0)       # IMU サニティ: 角速度上限 [rad/s]
        self.declare_parameter("imu_max_step_deg", 30.0)   # IMU サニティ: 姿勢跳躍上限 [deg]
        # 既定は検出のみで破棄しない。理由は imu_sanity.py 冒頭
        self.declare_parameter("imu_sanity_enforce", False)
        self.declare_parameter("publish", True)            # False = predict only, do not command
        # yaw の保持だけ切る。手で機体を回すと起動時の yaw へ戻ろうとして回り続けるため
        self.declare_parameter("hold_yaw", True)
        # duty_cycle の絶対値上限。1.0 = 制限なし。力は上限の 2 乗で効く (F = |u|^2 * 30 N)
        # ので 0.2 -> 0.4 は倍ではなく 4 倍。既定 0.25 の根拠は known_issues A-17
        self.declare_parameter("max_duty", 0.25)
        # 指令のレート制限。sim と揃える理由は thruster_limits.py 冒頭。0 以下で無効
        self.declare_parameter("servo_slew_deg_per_s", 250.0)
        self.declare_parameter("thrust_slew_per_s", 4.0)
        # 既定は disarmed。起動と同時にスラスタへ指令が出るのを避ける。
        # ~/arm サービス (data:true) でarm してから動かす。
        self.declare_parameter("start_armed", False)       # True = 起動と同時にarmする
        # Real-time setpoint (hold last; until a message arrives, use the launch defaults below).
        self.declare_parameter("setpoint_topic", "~/setpoint")   # umiusi_rl_control_msgs/AttitudeTarget
        # デッドマン: 速度指令が vel_timeout 秒来なければ 0 に戻す (姿勢目標は保持)。
        # 0 以下で無効、既定 off。使いどころは README
        self.declare_parameter("vel_timeout", 0.0)
        # --- 深度モード切替 (水圧センサ搭載時のみ。冒頭 docstring と depth_supervisor.py 参照) ---
        self.declare_parameter("depth_supervisor", False)  # true で有効化。max_duty 0.4 が前提
        # 起動時専用パラメータは read_only にする (レビュー指摘: 実行中の ros2 param set が
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
        self._active_model = None          # 直前の tick で使ったモデル (切替の検出用)
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
            if self._max_duty < 0.3:
                self.get_logger().warning(
                    f"depth_supervisor 有効だが max_duty={self._max_duty:.2f} — 深度試験は "
                    "**0.3 以上を推奨**。0.2 で降下できないのは上限ではなく零空間への配分が"
                    "原因で、0.4 は配分を直してから (issue #19)")

        self._sub_imu = self.create_subscription(
            Imu, self.get_parameter("imu_topic").value, self._on_imu, 1)
        # Real-time setpoint (optional): last message wins; absence keeps the defaults above.
        sp_topic = self.get_parameter("setpoint_topic").value
        self._sub_sp = self.create_subscription(AttitudeTarget, sp_topic, self._on_setpoint, 1)
        self._pubs = {p: self.create_publisher(ThrusterOutput, CMD_PREFIX + p, 10) for p in POSITIONS}
        # setpoint は type_mask で一部だけ更新されうるので、実際に適用中の値を別に出す
        self._pub_current_sp = self.create_publisher(
            AttitudeTarget, "~/current_setpoint", CURRENT_SETPOINT_QOS)
        self._arm = ArmState(self, self._detach_all,
                             start_armed=bool(self.get_parameter("start_armed").value))
        self._timer = self.create_timer(self._dt, self._tick)
        self.add_on_set_parameters_callback(self._on_set_params)
        self._publish_current_setpoint()
        # ポリシーは spin 前に読む。torch の import に数秒かかるので、tick 内で読むと
        # arm後の最初の周期で e-stop が止まる窓ができる
        self._ensure_model()
        self.get_logger().info(
            f"rl_attitude_node: default target=upright v_cmd=[{self._vel:.3f},0,0] m/s @ {self._hz:.0f} Hz "
            f"(publish={self._publish}); live setpoint (AttitudeTarget) on '{sp_topic}'")

    def _on_set_params(self, params):
        """ros2 param set を実行中に効かせる (hold_yaw / max_duty / vel_cmd / slew)。"""
        for p in params:
            if p.name == "hold_yaw":
                self._hold_yaw = bool(p.value)
                self.get_logger().info(
                    f"hold_yaw={self._hold_yaw}"
                    f"{'' if self._hold_yaw else ' — yaw は保持しません (roll/pitch のみ)'}")
            elif p.name == "max_duty":
                self._max_duty = abs(float(p.value))
                self.get_logger().info(f"max_duty={self._max_duty:.2f}")
                # 実行中の変更でも同じ検査をする。「現場で上限を上げたら速く動く」のが
                # 18 次元化の目的なので、実行中に変える導線こそ主流になる
                self._warn_if_max_duty_out_of_range()
                if self._sup_enabled and self._max_duty < 0.3:
                    self.get_logger().warning(
                        f"depth_supervisor 有効だが max_duty={self._max_duty:.2f} — "
                        "深度試験は 0.3 以上を推奨 (issue #19)")
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
            # ゼロ点は最初の ~25 サンプルの中央値 (水面で起動する前提)。取り直しは ~/zero_depth
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
        # BNO055 の化けサンプル対策。観測に直接入るので 1 発で指令が跳ねる (known_issues A-1)
        q, g = msg.orientation, msg.angular_velocity
        resyncs = self._imu_sanity.resyncs
        sample, reason = self._imu_sanity.update((q.w, q.x, q.y, q.z), (g.x, g.y, g.z))
        if reason is not None:
            self.get_logger().warning(self._imu_sanity.describe(reason),
                                      throttle_duration_sec=5.0)
        # enforce=False では reason が付いたまま採用されるので、独立した if で見る
        if self._imu_sanity.resyncs > resyncs:
            # 姿勢基準そのものが飛んだ。フィルタは再同期したが、目標姿勢は飛ぶ前の基準で
            # 与えられているので、目標を入れ直す必要がある。黙って進むと危険。
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
        """いま適用されている目標値を ~/current_setpoint に出す (latch)。

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
        if runner.obs_dim not in OBS_DIMS_WITH_VEL:
            # 水平と vert で次元が違ってよい。ただし 14 次元は速度指令を持たないので降下に使えない
            raise ValueError(
                f"vert モデルは速度指令を観測に持つタスク "
                f"({OBS_DIM} か {OBS_DIM_CAP} 次元) が必要です ({d}: {runner.obs_dim})")
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
        if self._obs_dim == OBS_DIM_CAP:
            self.get_logger().info(
                f"duty 上限を観測に持つポリシーです。max_duty={self._max_duty:.2f} を観測末尾に"
                "入れます — 実行中に `ros2 param set` で変えると方策の出力も追従します")
            self._warn_if_max_duty_out_of_range()
        self.get_logger().info(f"policy loaded from {d / 'export'} (obs {self._obs_dim}-D, rep103)")

    def _check_obs_fields(self, runner, export):
        """meta.json の obs_fields とこのノードの組み立て順を照合する。

        obs_fields は [["ori_err", 3], ["gyro", 3], ...] のような (名前, 幅) の並び。

        18 次元では必須。後方互換で警告だけにしてよいのは「既に出回っていて直せない
        バンドル」に限る話で、18 次元のバンドルはこの機能と同時に生まれたので守るべき既存が
        無い。しかも並びを取り違えて一番困るのが末尾に max_duty を足したこの次元なので、
        一番塞ぐべきところだけ穴が開くことになる。既存の 17/14 は警告のみで通す。
        """
        expected = OBS_FIELDS[runner.obs_dim]
        fields = runner.meta.get("obs_fields")
        if fields is None or len(fields) == 0:
            if runner.obs_dim == OBS_DIM_CAP:
                raise ValueError(
                    f"{export}/meta.json に obs_fields がありません。{OBS_DIM_CAP} 次元の"
                    "バンドルでは必須です (末尾の max_duty の位置を照合できないと、"
                    "golden が PASS しても方策が別の入力を読みます)。"
                    f"このノードの並び: {[n for n, _ in expected]}")
            self.get_logger().warning(
                f"{export}/meta.json に obs_fields がありません。観測の並びを照合できないので "
                "golden が PASS しても組み立て順の取り違えは検出できません "
                f"(このノードの並び: {[n for n, _ in expected]})")
            return
        try:
            got = tuple((str(n), int(w)) for n, w in fields)
        except (TypeError, ValueError) as e:
            # 形が違うものを黙って「照合できた」ことにしない。sim 側は幅の合計が合わない
            # ときは キーごと省略する 約束なので、ここに来るのは想定外の生成元。
            raise ValueError(
                f"obs_fields の形式が不正です ({export}/meta.json: {fields!r})。"
                '[["名前", 幅], ...] の並びが必要です') from e
        if sum(w for _, w in got) != runner.obs_dim:
            raise ValueError(
                f"obs_fields の幅の合計 {sum(w for _, w in got)} がポリシーの入力次元 "
                f"{runner.obs_dim} と一致しません ({export}/meta.json)")
        if got != expected:
            raise ValueError(
                f"観測レイアウトが sim と食い違っています ({export}/meta.json)。"
                f"バンドル: {got} / このノード: {expected}")
        self.get_logger().info(f"観測レイアウト一致: {[n for n, _ in expected]}")

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
        if runner.obs_dim not in OBS_DIMS_SUPPORTED:
            raise ValueError(
                f"対応していない観測次元 {runner.obs_dim} です ({export})。"
                f"{OBS_DIM_CAP} (attitude_velocity + duty 上限) / {OBS_DIM} (attitude_velocity) / "
                f"{OBS_DIM_NO_VEL} (attitude) のみ対応します "
                "(servo/thrust を観測に含む旧 25/22 次元ポリシーは廃止 — A-11 のエコー問題)")

        # 観測の組み立て順の検証。golden では PASS してしまう領域 (理由は README)
        self._check_obs_fields(runner, export)

        # 出力の契約 (action_mode)。ここも golden では検証できない — golden はネットの生出力を
        # 突き合わせるだけで、その 6 次元をどう 8 次元に直すかは見ていない。
        runner.mode_action = self._build_mode_action(runner, export)

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

    def _build_mode_action(self, runner, export):
        """action_mode を検証し、レンチモードなら ModeAction を、直接出力なら None を返す。

        ここで落とすのは配備前検証の一部。6 次元の生出力を 8 次元と取り違えたまま走ると、
        サーボ角と duty に無関係な値が入る (A-11 と同型の事故)。
        """
        mode = str(runner.meta.get("action_mode", ACTION_MODE_DIRECT))
        act_dim = int(runner.meta.get("act_dim", ACT_DIM))
        if mode == ACTION_MODE_DIRECT:
            if act_dim != ACT_DIM:
                raise ValueError(
                    f"action_mode={mode!r} なのに act_dim={act_dim} です ({export}/meta.json)。"
                    f"直接出力の方策は {ACT_DIM} 次元 [servo x4, esc x4] を出す約束です")
            return None
        if mode != ACTION_MODE_MODES:
            raise ValueError(
                f"未対応の action_mode={mode!r} です ({export}/meta.json)。"
                f"{ACTION_MODE_DIRECT!r} か {ACTION_MODE_MODES!r} のみ対応します")
        if act_dim != MODE_DIM:
            raise ValueError(
                f"action_mode={mode!r} なのに act_dim={act_dim} です ({export}/meta.json)。"
                f"レンチモードの方策は {MODE_DIM} 次元のモードレートを出す約束です")
        contract = runner.meta.get("action_contract")
        if not isinstance(contract, dict):
            raise ValueError(
                f"{export}/meta.json に action_contract がありません。レンチモードの方策は "
                "積分・ミキサ・折返しの係数を機械可読で持っている必要があります")
        ma = ModeAction(contract, POSITIONS)
        # ノードのパラメータと契約の食い違いは黙って丸めない。サーボ範囲がずれると
        # ミキサの正規化と _command の逆正規化が食い違い、角度が別物になる。
        if abs(ma.servo_range_deg - self._servo_range_deg) > 1e-6:
            raise ValueError(
                f"servo_range_deg がノード ({self._servo_range_deg}) と契約 "
                f"({ma.servo_range_deg}) で食い違っています ({export}/meta.json)")
        # 制御周期は物理時間で効く (m += a * slew * dt) ので dt は実測値を使う。ただし方策は
        # 「1 tick = 1/control_rate_hz」で学習しているので、ずれていれば応答が変わる。
        if ma.control_rate_hz > 0.0 and abs(ma.control_rate_hz - self._hz) > 1e-6:
            self.get_logger().warning(
                f"control_hz={self._hz:.1f} は学習時の {ma.control_rate_hz:.1f} Hz と違います。"
                "モードの積分は実測 dt で行うので発散はしませんが、応答は学習時と変わります")
        self.get_logger().info(
            f"action_mode=modes: モードレートを積分してミキサに通します "
            f"(slew {ma.mode_slew_per_s:.2f}/s, f_max = {ma.thrust_per_cmd:.0f} * "
            f"max_duty^{ma.thrust_curve_exp:.0f})。指令のレート制限は従来どおり掛けます")
        return ma

    def _obs_max_duty(self) -> float:
        """方策が観測する duty 上限。ミキサにも同じ値を渡す (モード 1.0 の意味を揃える)。"""
        return float(np.clip(self._max_duty, *MAX_DUTY_OBS_RANGE))

    def _warn_if_max_duty_out_of_range(self):
        """18 次元ポリシーで max_duty が学習分布の外なら警告する。

        観測に入る値はクランプするので方策が壊れることはないが、オペレータの意図と
        実挙動がずれる: 0.5 に上げても方策は 0.4 のつもりで指令を作る。黙って丸めない。
        """
        if self._obs_dim != OBS_DIM_CAP:
            return
        lo, hi = MAX_DUTY_OBS_RANGE
        if not (lo <= self._max_duty <= hi):
            self.get_logger().warning(
                f"max_duty={self._max_duty:.2f} は 18 次元ポリシーの学習分布 [{lo}, {hi}] の外です。"
                f"**観測に入れる値は {min(max(self._max_duty, lo), hi):.2f} にクランプ**します "
                "(duty のクリップ自体は設定値のまま)。方策は学習していない上限では意図どおりに"
                "動きません")

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
        if obs_dim in OBS_DIMS_WITH_VEL:      # attitude タスクだけが速度指令を持たない
            parts.append(v_cmd)
        parts.append(self._prev_action)
        if obs_dim == OBS_DIM_CAP:
            # duty 上限は末尾に足す。観測に入れる値だけ MAX_DUTY_OBS_RANGE へ丸める
            # (クリップの実値は _command 側の self._max_duty のまま)
            parts.append(np.array([self._obs_max_duty()], dtype=float))
        obs = np.concatenate(parts)
        if obs.shape[0] != obs_dim:           # レイアウトの取り違えを黙って通さない
            raise ValueError(
                f"観測を {obs.shape[0]} 次元で組みましたが、ポリシーは {obs_dim} 次元です")
        return obs

    def _supervise(self):
        """深度モード切替。-> (使うモデル, 観測に入れる v_cmd)。

        prev_action は共有のまま。水平と vert で観測次元が違ってよい (17 と 18 の混在) —
        観測はモデルごとに model.obs_dim で組み、prev_action の位置は
        どちらでも同じなのでずれない (test_obs_layout.py で固定)。
        ただし vert が 17 次元なら duty 上限の変化には追従しない (観測に持たないため)。
        """
        model, v_cmd = self._model, self._v_cmd
        if not (self._sup_enabled and self._vert_model is not None):
            return model, v_cmd
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._depth is None or self._depth_stamp is None or now - self._depth_stamp > 1.0:
            # 深度が古い: 鉛直成分だけ落として水平運転を続ける。状態機械もリセットする —
            # 古い brake/vert タイマーを残すと復帰時に watchdog が誤発火する
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
        self._select_model(model)
        # 鉛直指令インターロック: meta.json の vertical_ok が無いポリシーには z 成分を渡さない。
        # 水平専用に鉛直指令が入ると姿勢が崩壊する (depth_supervisor.py 冒頭)
        if v_cmd[2] != 0.0 and not model.meta.get("vertical_ok", False):
            self.get_logger().warning(
                "鉛直速度指令を 0 にクランプしました — このポリシーは水平専用です "
                "(vertical_ok なし)。降下バーストは av_cal5_3d_rep103 で",
                throttle_duration_sec=5.0)
            v_cmd = np.array([v_cmd[0], v_cmd[1], 0.0])
        raw = model.act(self._build_obs(v_cmd, model.obs_dim))
        if model.mode_action is None:
            action = np.clip(np.asarray(raw, dtype=float).reshape(ACT_DIM), -1.0, 1.0)
        else:
            # レンチモード: 6 次元のレートを 8 次元に直す。max_duty は方策が観測しているのと
            # 同じ値を渡すこと (README「レンチモード action を使う側のルール」)
            action = model.mode_action.step(raw, self._obs_max_duty(), self._dt)
        if self._publish:
            self._command(action)
        # 観測に返すのは ミックス後の 8 次元 (sim の prev_action と同じもの)
        self._prev_action = action

    def _command(self, action):
        # sim のプラントが持つレート制限の再現。レンチモードの方策でも外さない。
        # control の max_duty_step_per_sec は /cmd/direct を素通りする (known_issues B-12)
        # ので、ここで掛けなければ誰も掛けない (known_issues A-11)
        servo_target = np.asarray(action[:4], dtype=float) * self._servo_range_deg
        duty_target = np.clip(np.asarray(action[4:], dtype=float), -self._max_duty, self._max_duty)
        self._servo_cmd = slew(self._servo_cmd, servo_target, self._servo_slew, self._dt)
        self._duty_cmd = slew(self._duty_cmd, duty_target, self._thrust_slew, self._dt)
        for k, p in enumerate(POSITIONS):
            out = ThrusterOutput()
            out.runnable = ThrusterRunnable(esc=True, servo=True)
            out.duty_cycle = float(self._duty_cmd[k])
            # 単位は度 (known_issues B-13)。ch 別のサーボ符号はこの境界でだけ当てる
            # (_servo_cmd は sim 規約のまま保つ)。範囲外は CAN 送信が失敗するので ±90 に収める
            out.angle = max(-90.0, min(90.0, float(self._servo_cmd[k]) * self._servo_sign[k]))
            self._pubs[p].publish(out)

    def _detach_all(self):
        """DISARM: zero output + runnable false on every thruster -> the control stack detaches
        esc/servo (hardware-level not-allowed). The e-stop / disarm path for the direct loop."""
        # 下の早期 return より前に置くこと。compute-only でも prev_action がずれる
        # (初期化の中身は test_model_switch.py が固定)
        self._reset_mode_state()
        if not self._publish:      # compute-only node never commands /cmd, so nothing to detach
            return
        # 停止はレート制限を通さない (安全側。次にarm したとき 0 から積み直す)
        self._servo_cmd[:] = 0.0
        self._duty_cmd[:] = 0.0
        for p in POSITIONS:
            out = ThrusterOutput()
            out.runnable = ThrusterRunnable(esc=False, servo=False)
            out.duty_cycle = 0.0
            out.angle = 0.0
            self._pubs[p].publish(out)

    def _select_model(self, model):
        """今 tick で使うモデルを確定する。切り替わったら、これから使うほうの積分器を 0 に戻す。

        モードの積分器はモデルごとに持つので、深度モードの切替で待機していた側は
        「最後に使ったときのモードベクトル」を抱えたままになる。再選択の瞬間にその力が
        いきなり出るのは危険で、sim には対応する状況が無い (env は方策 1 個・積分器 1 個)。
        リセットするのは新しく選ばれたほう — 出ていく側の状態は、それ自身が再選択される
        ときにこの判定で消えるので触らなくてよい。
        現状はどちらのモデルも direct 出力なので不活性だが、vert に modes 系を載せた瞬間に効く。
        """
        if model is self._active_model:
            return
        if model.mode_action is not None:
            model.mode_action.reset()
        self._active_model = model

    def _reset_mode_state(self):
        """レンチモードの積分器と前回サーボ角を初期化する (disarm / e-stop のたび)。

        prev_action も 0 に戻す — 観測の proprio は「自分が直前に出した指令」なので、
        指令を出していない間の値を残すと、再 armした最初の観測が実際とずれる。
        """
        reset_any = False
        for m in (self._model, self._vert_model):
            if m is not None and getattr(m, "mode_action", None) is not None:
                m.mode_action.reset()
                reset_any = True
        self._active_model = None      # 再 arm時に切替判定をやり直す
        if reset_any:
            # 直接出力の方策では従来どおり prev_action を触らない (挙動を変えない)
            self._prev_action = np.zeros(ACT_DIM)

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
