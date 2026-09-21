#!/usr/bin/env python3
"""古典制御 (姿勢 PID + 幾何アロケーション) の姿勢制御ノード。

**制御則はここに書かない。** `umiusi_perception.classical` を import して呼ぶだけの薄い
ラッパにする。同じコードを sim と実機で動かすのが目的で、書き直すと両者がずれる
(記録に残る「sim では正しいが実機で変」は 4 件すべて軸や系の取り違えだった)。

`rl_attitude_node` と同じ topic / param 名を使う。方策の差し替えとして見えるほうが
運用の間違いが少ないため。**同時に起動しないこと** — 同じ `/cmd/direct` を取り合う。

必要なもの:
  * `umiusi_perception` wheel (classical を含むもの)。
    `pip install --no-deps --no-index <sim>/packages/perception`
  * `tools/export_classical.py` が書き出した `classical_bundle.json`
    (`bundle_path` で渡す。既定は同梱のもの)

座標系は 2 つあり、**変換は umiusi_perception の関数だけを使う**。ここで軸を入れ替えない。
"""

from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import Imu
from sinsei_umiusi_msgs.msg import ThrusterOutput, ThrusterRunnable

from umiusi_common import thrust_sign
from umiusi_common.arm import ArmState
from umiusi_common.imu_sanity import ImuSanity
from umiusi_rl_control_msgs.msg import AttitudeTarget

POSITIONS = ("lf", "lb", "rb", "rf")
CMD_PREFIX = "/cmd/direct/thruster_controller/output_"
YAW_IDX = 2          # ori_err / gyro は REP-103 (x 前, y 左, z 上) なので yaw は添字 2
CURRENT_SETPOINT_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
# 制御の計算が何周期続けて失敗したら disarm するか。50 Hz なので 10 = 0.2 秒。
# 1 サンプルの化けで run を終わらせず、壊れ続けるなら止める、の折り合い
FAIL_LIMIT = 10


def _wrap(a):
    """角度を (-pi, pi] に畳む。"""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


YAW_DT_MAX = 0.2     # [s] 指令が途切れたときに方位目標が飛ばないための上限


def advance_yaw_setpoint(yaw_sp, yaw_now, rz, rate_scale, dt, lead_max):
    """旋回レート指令 `rz` を方位目標へ積分する (`cmd_target_yaw_mode="rate"`)。

    呼び出し側の義務:
      * `dt` = **前の指令から今までの実時間**。制御周期を渡すと旋回速度が指令の
        送信レートに比例してずれる (known_issues B-17)
      * `lead_max` = 実測方位からの先行量の上限 (known_issues B-14 の 180 度の罠)
    """
    if yaw_sp is None:
        yaw_sp = yaw_now
    yaw_sp = _wrap(yaw_sp + rz * rate_scale * dt)
    lead = _wrap(yaw_sp - yaw_now)
    if abs(lead) > lead_max:      # 追随できない目標を先行させない
        yaw_sp = _wrap(yaw_now + math.copysign(lead_max, lead))
    return yaw_sp


def slew(current, target, max_rate, dt):
    """1 ステップで `max_rate * dt` だけ target に近づける。`max_rate <= 0` で無制限。"""
    if max_rate <= 0.0:
        return np.asarray(target, dtype=float)
    step = max_rate * dt
    return current + np.clip(np.asarray(target, dtype=float) - current, -step, step)


def sub_quat(q_target, q_imu):
    """目標姿勢までの回転ベクトル [rad]。`ori_err` の定義。両方 (w, x, y, z)。"""
    tw, tx, ty, tz = q_target
    w, x, y, z = q_imu
    # q_err = conj(q_imu) * q_target
    ew = w * tw + x * tx + y * ty + z * tz
    ex = w * tx - x * tw - y * tz + z * ty
    ey = w * ty + x * tz - y * tw - z * tx
    ez = w * tz - x * ty + y * tx - z * tw
    if ew < 0.0:  # q と -q は同じ姿勢。短い側を取る
        ew, ex, ey, ez = -ew, -ex, -ey, -ez
    n = math.sqrt(ex * ex + ey * ey + ez * ez)
    if n < 1e-12:
        return np.zeros(3)
    return np.array([ex, ey, ez]) * (2.0 * math.atan2(n, ew) / n)


class ClassicalAttitudeNode(Node):
    def __init__(self):
        super().__init__("classical_attitude")

        self.declare_parameter("bundle_path", "")
        self.declare_parameter("imu_topic", "/state/imu")
        self.declare_parameter("control_hz", 50.0)
        self.declare_parameter("max_duty", 0.25)
        self.declare_parameter("vel_cmd", 0.0)
        self.declare_parameter("publish", True)
        self.declare_parameter("start_armed", False)
        self.declare_parameter("setpoint_topic", "~/setpoint")
        # **`/cmd/target` を目標として受ける** (空文字で無効)。core の
        # manual_target_generator が UI (ゲームパッド) の `/user_input/target` をそのまま
        # 中継してくるトピックで、FSM の navigator も `command_mode:=target` でここへ出す。
        #
        # これを有効にすると、**UI のテレオペが姿勢制御の上に乗る** — 操縦者は速度と
        # 向きだけ出し、roll/pitch の安定化と 4 基への配分はこのノードがやる。
        # control 側の logic は `/cmd/direct` に publisher が居る間スキップされる
        # (`thruster_controller.cpp` の `has_no_thruster_publishers`) ので、
        # このノードが上がっていれば二重駆動にはならない。
        #
        # **`Target.orientation` の解釈は control の `logic::attitude::Rl` に合わせる**:
        # REP-103 の**回転ベクトル [rad]** (大きさ = 回転角、向き = 回転軸、ゼロ = 水平)。
        # 同じフィールドを control の `ff` はトルク配分の入力として、autonomy の
        # navigator (`command_mode:=target`) は正規化した指令として使っており、
        # **意味が 3 通りに割れている**。ここは 3 つのうち唯一絶対量として定義されている
        # rl の規約を採る。navigator の target 経路と混ぜる前に揃えること
        self.declare_parameter("cmd_target_topic", "")
        # --- UI の実際の出力に合わせる 2 つのつまみ ------------------------------------
        # UI (`sinsei_UMIUSI_ui/src/services/gamepadPublisher.ts`) が出すのは
        # **正規化したスティック値**で、物理単位ではない:
        #     velocity.x = -左スティック上下 ([-1,1])   velocity.y = 十字キーで ±0.5
        #     velocity.z = L2/R2 で ±0.3
        #     orientation = 右スティックに 0.3 / 左スティック左右に -0.2 を掛けたもの
        # 一方この制御器は velocity を **m/s** で取り、到達速度は cap 0.25 で約 0.21 m/s。
        # 素で渡すとスティックを倒した瞬間に到達速度の 5 倍を指令することになり、常に
        # 振り切り状態になる。**到達速度を 1.0 に対応させる**のがこの係数 (0 以下で無効)。
        self.declare_parameter("cmd_target_vel_scale", -1.0)
        # yaw の扱い。"absolute" = control の `logic::attitude::Rl` と同じ、回転ベクトルの
        # z 成分を**絶対角 [rad]** として読む。UI の既定だと ±0.2 rad = ±11 度しか回れず、
        # ゲームパッドの操作としては使いものにならない。"rate" は z を**旋回レート**として
        # 積分し、方位目標を作る (navigator の setpoint 経路と同じ扱い・同じ歯止め)。
        # roll/pitch は "rate" でも絶対角のまま — 倒した角度に傾き、放せば水平に戻る
        self.declare_parameter("cmd_target_yaw_mode", "absolute")
        self.declare_parameter("cmd_target_yaw_rate_scale", 1.5)   # [rad/s] per unit
        self.declare_parameter("cmd_target_yaw_lead_max", 1.05)    # 60 deg
        # `/cmd/direct` は control の max_duty もスルーレート制限も素通りする
        # (known_issues B-12) ので、ここで掛けなければ誰も掛けない (known_issues A-11)。
        # 古典制御の特異点回避はこのサーボ速度限界を前提に調整してある
        self.declare_parameter("servo_slew_deg_per_s", 250.0)
        self.declare_parameter("thrust_slew_per_s", 4.0)
        self.declare_parameter("servo_sign", [1.0, 1.0, 1.0, 1.0])
        # **推力の向き**。基ごとに duty へ掛ける (-1.0 で反転)。順序は POSITIONS。
        # 2026-09-12 のプール run で、意図したモーメントと実測の角加速度が 3 軸とも逆相関し
        # (yaw -0.77〜-0.87)、arm すると roll/pitch の振れが 2〜3 倍に悪化した — 制御が浮力の
        # 復元を打ち消していた。原因は指令が機体に届いたあと (ペラの回転方向 / モータ相 /
        # ESC の逆転設定) にあり、ソフト側は自己整合していた (bag の再生が記録と完全一致)。
        # **現場でコードを触らずに直せるようにここに置く。** どの基が反転しているかは
        # tools/thrust_sign_check.py が 1 基ずつ測って出す。`ros2 param set` で実行中に変更可。
        self.declare_parameter("thrust_sign", [1.0, 1.0, 1.0, 1.0])
        # 極性の唯一の正をどこに置くか。"control" = control の
        # `thruster_controller_<pos>.is_forward` に合わせに行く (既定。control の yaml を
        # 直せば autonomy にも効く)。"param" = 上の thrust_sign をそのまま使う
        # (control が居ない sim bridge / ベンチ用)。詳細は umiusi_common/thrust_sign.py
        self.declare_parameter("thrust_sign_source", "control")
        # false で yaw の保持だけ切る (roll/pitch のみ保つ)。rl_attitude_node と同じ名前・既定。
        # 目標 yaw は IMU の基準方位であって現場の向きではないので、方位を保たせたくない
        # ときや、yaw の符号だけ怪しいときの切り分けに使う。実行中も param set で切替可
        self.declare_parameter("hold_yaw", True)
        # **死んだスラスタを配分から外す** (lf, lb, rb, rf の順。false = そのスラスタは無い扱い)。
        # `GeneralAllocator` は生きている列だけで解き直すので、**1 基欠損でも 6 自由度の
        # 権限は残る** — 縮約した 6x8 の rank は 6、条件数は 5.34 -> 8.05 (lf 欠損、実測)。
        # 外さないと「死んだ基に指令を出し続け、実際に出る力は解いたものと違う」状態になり、
        # 残り 3 基が誤った前提で釣り合いを取るので姿勢が崩れる。
        # **余裕は無くなる**ので cap を上げるか、機動をゆっくりにすること。
        # `ros2 param set` で実行中に変更可。**RL 側にはこの機能は無い** (方策は 4 基前提)。
        self.declare_parameter("live_thrusters", [True, True, True, True])
        # 生死の正をどこに置くか。"control" = control の `thruster_controller_<pos>.esc_disabled`
        # を読みに行く (既定)。**`esc_disabled` は /cmd/direct にも効く**ので、control で
        # 殺したのに autonomy が知らないと「死んだ基に配分し続ける」状態になる。
        # "param" = 上の live_thrusters をそのまま使う (control が居ないとき)
        self.declare_parameter("live_thrusters_source", "control")
        # 鉛直 (heave) の速度フィードバックゲイン。**0 = 前進項だけ**（指令した鉛直速度を
        # 抗力に対して開ループで保つ）。負でバンドルの gains の値を使う。
        # 深度センサではなく指令からの推測 (VelocityObserver) を見るので、上げるのは
        # 鉛直の推定が当てになると確かめてから。`ros2 param set` で実行中に変更可
        self.declare_parameter("k_v_vert", -1.0)
        # --- 断の検出。**どちらも「黙って走り続ける」のを防ぐためのもの** -----------------
        # IMU が途切れたら姿勢制御は成立しない (凍った推定で一定トルクを出し続けて回り出す)。
        # 途切れたら出力 0。**disarm はしない** — 正浮力なので 0 出力は浮上側に外れる。
        # 0 以下で無効。`ImuSanity` は 1 サンプルの化けを見るだけで断は見ない
        self.declare_parameter("imu_timeout", 1.0)
        # デッドマン: 速度指令がこの秒数来なければ**並進を 0 にする**(姿勢目標は保持)。
        # 2026-09-13 の bag では UI 側が更新を止めていた (`Target is not updated` 421 件)。
        # 0 以下で無効。rl_attitude_node の同名パラメータと意味を揃えてある
        self.declare_parameter("vel_timeout", 1.0)
        self.declare_parameter("imu_max_gyro", 10.0)
        self.declare_parameter("imu_max_step_deg", 30.0)
        self.declare_parameter("imu_sanity_enforce", False)

        self._dt = 1.0 / float(self.get_parameter("control_hz").value)
        self._max_duty = abs(float(self.get_parameter("max_duty").value))
        self._publish = bool(self.get_parameter("publish").value)
        self._servo_slew = float(self.get_parameter("servo_slew_deg_per_s").value)
        self._thrust_slew = float(self.get_parameter("thrust_slew_per_s").value)
        self._servo_sign = self._signs("servo_sign")
        self._thrust_sign = thrust_sign.resolve(
            self, POSITIONS, str(self.get_parameter("thrust_sign_source").value).strip(),
            self._signs("thrust_sign"))
        self._hold_yaw = bool(self.get_parameter("hold_yaw").value)

        if not self._load(self.get_parameter("bundle_path").value):
            raise SystemExit(1)

        self._sanity = ImuSanity(
            max_gyro=float(self.get_parameter("imu_max_gyro").value),
            max_step_deg=float(self.get_parameter("imu_max_step_deg").value),
            enforce=bool(self.get_parameter("imu_sanity_enforce").value))

        self._imu_timeout = float(self.get_parameter("imu_timeout").value)
        self._vel_timeout = float(self.get_parameter("vel_timeout").value)
        self._fail = 0            # 連続して制御の計算に失敗した周期数
        self._imu_t = None        # 最後に IMU を受けた時刻。None = 一度も来ていない
        self._vel_t = None        # 最後に速度指令を受けた時刻。None = デッドマン未武装
        self._vel_dead = False

        self._quat = np.array([1.0, 0.0, 0.0, 0.0])   # (w, x, y, z)
        self._gyro = np.zeros(3)
        self._target = np.array([1.0, 0.0, 0.0, 0.0])
        self._vel_cmd = np.array([float(self.get_parameter("vel_cmd").value), 0.0, 0.0])
        self._servo_cmd = np.zeros(4)
        self._duty_cmd = np.zeros(4)
        self._action = np.zeros(8)     # 直前に出した指令 (servo ±1, duty)。観測器が使う

        self._pubs = {p: self.create_publisher(ThrusterOutput, CMD_PREFIX + p, 10)
                      for p in POSITIONS}
        self.create_subscription(
            Imu, self.get_parameter("imu_topic").value, self._on_imu, 10)
        self.create_subscription(
            AttitudeTarget, self.get_parameter("setpoint_topic").value,
            self._on_setpoint, CURRENT_SETPOINT_QOS)
        self._vel_scale = float(self.get_parameter("cmd_target_vel_scale").value)
        if self._vel_scale < 0.0:
            # 負 = 自動。スティックの 1.0 を「その cap で到達できる速度」に対応させる
            self._vel_scale = self._ctl.reachable_speed(self._max_duty)
        self._yaw_mode = str(self.get_parameter("cmd_target_yaw_mode").value).strip().lower()
        self._yaw_rate_scale = float(self.get_parameter("cmd_target_yaw_rate_scale").value)
        self._yaw_lead_max = abs(float(self.get_parameter("cmd_target_yaw_lead_max").value))
        self._yaw_sp = None
        self._yaw_t_prev = None
        cmd_target = str(self.get_parameter("cmd_target_topic").value).strip()
        if cmd_target:
            from sinsei_umiusi_msgs.msg import Target
            self.create_subscription(Target, cmd_target, self._on_cmd_target, 10)
            self.get_logger().info(
                f"'{cmd_target}' (Target) も目標として受ける — UI のテレオペが"
                "姿勢制御の上に乗る。orientation は REP-103 の回転ベクトル [rad] と解釈する")
            if self._yaw_mode != "rate":
                # UI が出す orientation.z は正規化したスティック値なので、absolute だと
                # そのまま絶対方位 [rad] になる (known_issues B-17)
                self.get_logger().warning(
                    f"cmd_target_yaw_mode='{self._yaw_mode}' で '{cmd_target}' を受けている。"
                    "**UI のゲームパッドなら ±11 度しか回れない** "
                    "(orientation.z を絶対方位として読むため)。旋回させるなら "
                    "`ros2 param set /classical_attitude cmd_target_yaw_mode rate`")

        self._arm = ArmState(self, self._detach_all,
                             start_armed=bool(self.get_parameter("start_armed").value))
        self.add_on_set_parameters_callback(self._on_params)
        self.create_timer(self._dt, self._tick)
        self.get_logger().info(
            f"classical attitude: {self._dt * 1000:.0f} ms, max_duty={self._max_duty:.2f}, "
            f"publish={self._publish}, hold_yaw={self._hold_yaw}, "
            f"thrust_sign={self._thrust_sign}, servo_sign={self._servo_sign}, "
            f"imu_timeout={self._imu_timeout:.1f}s, vel_timeout={self._vel_timeout:.1f}s")
        if abs(float(self.get_parameter("vel_cmd").value)) > 1e-9:
            # `vel_cmd` は相手の居ない静的な巡航指令なので、**デッドマンでは守れない**
            # (生存を確認する相手が居ない)。黙って守れていない状態にしない
            self.get_logger().warning(
                f"vel_cmd={float(self.get_parameter('vel_cmd').value):.2f} m/s で起動した。"
                "**この指令はデッドマンの保護外** — 止めるのは disarm / e-stop だけ")

    def _apply_live(self):
        """`live_thrusters` をアロケータへ当てる。**全滅と 2 基以上の欠損は拒否する** —
        2 基欠けると 6 自由度の権限が無くなり、解いた wrench が出ない (黙って姿勢が崩れる)。"""
        v = [bool(x) for x in self.get_parameter("live_thrusters").value]
        if len(v) != len(POSITIONS):
            raise ValueError(f"live_thrusters は {POSITIONS} と同じ {len(POSITIONS)} 個が要る: {v}")
        if str(self.get_parameter("live_thrusters_source").value).strip() != "param":
            v = thrust_sign.live_from_control(self, POSITIONS, v)
        dead = [p for p, a in zip(POSITIONS, v) if not a]
        if len(dead) > 1:
            self.get_logger().error(
                f"死亡扱いが {dead} と 2 基以上 — **6 自由度の権限が無くなる**ので受け付けない。"
                "1 基までにすること")
            return False
        self._alloc.set_live(v)
        self._alloc.reset()
        if dead:
            self.get_logger().warning(
                f"**{dead[0]} を死亡扱いにした。** 残り 3 基で解き直す (6 自由度は保つが"
                "余裕は無い)。cap を上げるか機動をゆっくりに。RL 側は 4 基前提なので使えない")
        return True

    def _signs(self, name):
        """符号パラメータを読む。**長さが違えば起動させない** — 3 個しか書かないミスは
        黙って 1 基ぶん既定のまま残り、水中で初めて気付くことになる。"""
        v = [float(x) for x in self.get_parameter(name).value]
        if len(v) != len(POSITIONS):
            raise ValueError(f"{name} は {POSITIONS} と同じ {len(POSITIONS)} 個が要る: {v}")
        return v

    def _on_params(self, params):
        """`ros2 param set` を実行中に効かせる。**現場でコードを触らずに符号を直すための導線。**

        再起動を挟むと arm し直しになり、そのぶん機体が漂う。反転の切り分けは試行回数が要る
        ので、走らせたまま変えられることに意味がある。
        """
        from rcl_interfaces.msg import SetParametersResult
        for p in params:
            try:
                if p.name in ("thrust_sign", "servo_sign"):
                    v = [float(x) for x in p.value]
                    if len(v) != len(POSITIONS):
                        return SetParametersResult(
                            successful=False,
                            reason=f"{p.name} は {len(POSITIONS)} 個 {POSITIONS} が要る")
                    setattr(self, "_" + p.name, v)
                    self.get_logger().warning(f"{p.name}={v}")
                elif p.name == "hold_yaw":
                    self._hold_yaw = bool(p.value)
                    self.get_logger().warning(
                        f"hold_yaw={self._hold_yaw}"
                        f"{'' if self._hold_yaw else ' — yaw は保持しません (roll/pitch のみ)'}")
                elif p.name == "max_duty":
                    self._max_duty = abs(float(p.value))
                    # 速度スケールが「到達速度」由来なら cap に追随させる。置いていくと
                    # cap を上げたのにスティック 1.0 の意味が古いままになる
                    if (float(self.get_parameter("cmd_target_vel_scale").value) < 0.0
                            and getattr(self, "_ctl", None) is not None):
                        self._vel_scale = self._ctl.reachable_speed(self._max_duty)
                    self.get_logger().warning(
                        f"max_duty={self._max_duty:.2f} (vel_scale={self._vel_scale:.3f})")
                elif p.name == "live_thrusters":
                    # 先に宣言値を書き換えてから当てる (バリデーションは _apply_live 側)
                    v = [bool(x) for x in p.value]
                    if len(v) != len(POSITIONS) or sum(1 for a in v if not a) > 1:
                        return SetParametersResult(
                            successful=False,
                            reason=f"live_thrusters は {len(POSITIONS)} 個・死亡は 1 基まで")
                    self._alloc.set_live(v)
                    self._alloc.reset()
                    dead = [q for q, a in zip(POSITIONS, v) if not a]
                    self.get_logger().warning(
                        f"live_thrusters={v}" + (f" — {dead[0]} を外した" if dead else ""))
                elif p.name == "live_thrusters_source":
                    src = str(p.value).strip().lower()
                    if src not in ("control", "param"):
                        return SetParametersResult(
                            successful=False,
                            reason=f"live_thrusters_source は control か param: '{p.value}'")
                    # **ここでは control へ読みに行かない** — パラメータコールバックの中で
                    # サービスを待つと spin が入れ子になって固まる。効くのは次に
                    # live_thrusters を set したときか、再起動したとき
                    self.get_logger().warning(
                        f"live_thrusters_source={src} — **この set だけでは読み直さない。** "
                        "続けて `ros2 param set /classical_attitude live_thrusters '[...]'` "
                        "を実行するか、再起動すること")
                elif p.name == "k_v_vert":
                    # 制御器は属性を持つだけなので、作り直さずその場で差し替えられる
                    ctl = getattr(self, "_ctl", None)
                    if ctl is not None and float(p.value) >= 0.0:
                        ctl.k_v_vert = float(p.value)
                        self.get_logger().warning(f"k_v_vert={float(p.value):.2f}")
                elif p.name == "vel_cmd":
                    self._vel_cmd = np.array([float(p.value), 0.0, 0.0])
                    # **デッドマンは叩かない。** これは相手の居ない静的な指令で、
                    # 生存を確認しようが無い。叩くと vel_timeout 秒で勝手に 0 になる
                    self.get_logger().warning(
                        f"vel_cmd={float(p.value):.2f} m/s"
                        + (" — **デッドマンの保護外**" if abs(float(p.value)) > 1e-9 else ""))
                elif p.name in ("imu_timeout", "vel_timeout"):
                    setattr(self, "_" + p.name, float(p.value))
                    self.get_logger().warning(
                        f"{p.name}={float(p.value):.1f} s"
                        + (" — **無効**" if float(p.value) <= 0.0 else ""))
                # --- テレオペのつまみ。**起動時に読んだきりにしない** ---------------
                # 手順書 (docs/teleop_gamepad.md, docs/field_card.md) が現場で
                # `ros2 param set` するよう書いているもの。受けないと set は成功を
                # 返すのに何も変わらない (known_issues B-17)
                elif p.name == "cmd_target_yaw_mode":
                    mode = str(p.value).strip().lower()
                    if mode not in ("absolute", "rate"):
                        return SetParametersResult(
                            successful=False,
                            reason=f"cmd_target_yaw_mode は absolute か rate: '{p.value}'")
                    self._yaw_mode = mode
                    # 溜めた方位目標と受信時刻は持ち越さない (モードが変わった瞬間に
                    # 古い目標へ飛ぶ / 切替の間隔をまとめて積むのを防ぐ)
                    self._yaw_sp = None
                    self._yaw_t_prev = None
                    self.get_logger().warning(
                        f"cmd_target_yaw_mode={mode}"
                        + (" — orientation.z を旋回レートとして積む"
                           if mode == "rate" else
                           " — orientation.z は絶対方位 [rad]。**UI だと ±11 度しか回れない**"))
                elif p.name == "cmd_target_yaw_rate_scale":
                    self._yaw_rate_scale = float(p.value)
                    self.get_logger().warning(
                        f"cmd_target_yaw_rate_scale={self._yaw_rate_scale:.2f} rad/s per unit")
                elif p.name == "cmd_target_yaw_lead_max":
                    self._yaw_lead_max = abs(float(p.value))
                    self.get_logger().warning(
                        f"cmd_target_yaw_lead_max={math.degrees(self._yaw_lead_max):.0f} deg")
                elif p.name == "cmd_target_vel_scale":
                    v = float(p.value)
                    ctl = getattr(self, "_ctl", None)
                    if v < 0.0 and ctl is None:
                        return SetParametersResult(
                            successful=False, reason="制御器が未構築で自動値を計算できない")
                    # 負 = 自動 (スティック 1.0 = その cap で到達できる速度)。init と同じ規則
                    self._vel_scale = ctl.reachable_speed(self._max_duty) if v < 0.0 else v
                    self.get_logger().warning(
                        f"cmd_target_vel_scale={self._vel_scale:.3f} m/s per unit"
                        + (" (自動)" if v < 0.0 else ""))
            except (TypeError, ValueError) as e:            # noqa: PERF203
                return SetParametersResult(successful=False, reason=f"{p.name}: {e}")
        return SetParametersResult(successful=True)

    def _load(self, path):
        """バンドルを読んで制御器を組む。**制御則はここに無い** — classical を呼ぶだけ。"""
        try:
            from umiusi_perception.classical import (ClassicalController, GeneralAllocator,
                                                     PlantContract)
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(
                f"umiusi_perception.classical を import できません ({type(e).__name__}: {e})。"
                "pip install --no-deps --no-index <sim>/packages/perception で入れること")
            return False
        if not path:
            share = Path(get_package_share_directory("umiusi_autonomy"))
            path = str(share / "config" / "classical_bundle.json")
        if not Path(path).exists():
            self.get_logger().error(
                f"バンドルがありません: {path} "
                "(umiusi_sim の tools/export_classical.py で書き出す)")
            return False
        b = json.loads(Path(path).read_text())
        plant = PlantContract.from_dict(b["contract"])
        gains = dict(b.get("gains", {}))
        kvv = float(self.get_parameter("k_v_vert").value)
        if kvv >= 0.0:
            gains["k_v_vert"] = kvv
        self._ctl = ClassicalController(plant, **gains)
        self._alloc = GeneralAllocator(plant, **b.get("allocator", {}))
        self._apply_live()
        # 較正前のバンドルで絶対値を信じないこと。相対比較には使える
        for k in b.get("uncalibrated", []):
            self.get_logger().warn(f"未較正の契約値: {k}")
        self.get_logger().info(
            f"bundle: {path} (cap_ref={plant.cap_ref:.2f}, "
            f"{plant.control_rate_hz:.0f} Hz, 到達速度 @max_duty="
            f"{self._ctl.reachable_speed(self._max_duty):.3f} m/s)")
        return True

    def _now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def _on_imu(self, msg):
        q = msg.orientation
        g = msg.angular_velocity
        sample, _reason = self._sanity.update((q.w, q.x, q.y, q.z), (g.x, g.y, g.z))
        if sample is None:
            return
        self._imu_t = self._now()
        self._quat = np.asarray(sample.quat, dtype=float)
        self._gyro = np.asarray(sample.gyro, dtype=float)

    def _on_setpoint(self, msg):
        self._vel_t = self._now()
        o = msg.orientation
        self._target = np.array([o.w, o.x, o.y, o.z], dtype=float)
        v = msg.velocity
        self._vel_cmd = np.array([v.x, v.y, v.z], dtype=float)

    def _on_cmd_target(self, msg):
        """`/cmd/target` (sinsei_umiusi_msgs/Target) を目標に変換する。

        `orientation` は **REP-103 の回転ベクトル [rad]** (control の `logic::attitude::Rl`
        と同じ規約)。大きさが回転角、向きが回転軸、ゼロが水平・方位維持。
        `cmd_target_yaw_mode="rate"` のときだけ z 成分を旋回レートとして積分する
        (ゲームパッド用。理由はパラメータ宣言のコメント)。
        """
        o, v = msg.orientation, msg.velocity
        rx, ry, rz = float(o.x), float(o.y), float(o.z)

        if self._yaw_mode == "rate":
            yaw_now = self._yaw_now()
            if yaw_now is None:
                return                       # IMU 待ち: 目標を作れない
            # 積分は受信間隔の実時間で行う (advance_yaw_setpoint)。初回は制御周期で
            # 代用し、途切れたときは YAW_DT_MAX で頭打ちにする
            now = self.get_clock().now().nanoseconds * 1e-9
            dt = self._dt if self._yaw_t_prev is None else now - self._yaw_t_prev
            self._yaw_t_prev = now
            self._yaw_sp = advance_yaw_setpoint(
                self._yaw_sp, yaw_now, rz, self._yaw_rate_scale,
                min(max(dt, 0.0), YAW_DT_MAX), self._yaw_lead_max)
            rz = _wrap(self._yaw_sp)         # 以降は絶対角として扱う

        rv = np.array([rx, ry, rz], dtype=float)
        theta = float(np.linalg.norm(rv))
        if theta < 1e-12:
            self._target = np.array([1.0, 0.0, 0.0, 0.0])
        else:
            s = math.sin(theta / 2.0) / theta
            self._target = np.array([math.cos(theta / 2.0), *(rv * s)])

        # UI は正規化したスティック値を送ってくるので m/s へ直す。係数が負なら素通し
        # (FSM や自前のノードが m/s で出しているとき)
        k = self._vel_scale if self._vel_scale > 0.0 else 1.0
        self._vel_cmd = np.array([v.x, v.y, v.z], dtype=float) * k
        self._vel_t = self._now()

    def _yaw_now(self):
        """IMU の現在方位 [rad]。まだ受けていなければ None。"""
        w, x, y, z = self._quat
        if abs(float(np.dot(self._quat, self._quat)) - 1.0) > 0.5:
            return None
        return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    def _imu_stale(self):
        """IMU が途切れているか。**途切れたら姿勢制御は成立しない。**

        凍った姿勢推定で回し続けると、誤差が一定のまま同じトルクを出し続けて機体が
        回り始める (誤差が縮まないので減りもしない)。`ImuSanity` は 1 サンプルの化けを
        見るだけで**断は見ない**ので、ここで見る。
        """
        if self._imu_timeout <= 0.0:
            return False
        return self._imu_t is None or (self._now() - self._imu_t) > self._imu_timeout

    def _vel_stale(self):
        """速度指令が途切れているか (デッドマン)。

        途切れても**姿勢目標は保つ**。止めるのは並進だけ — 出力ごと切ると正浮力で
        浮上するので、操縦者が意図していない動きになる。実機では UI 側が更新を止めた
        ことが実際にあった (2026-09-13 の bag に `Target is not updated` が 421 件)。
        """
        if self._vel_timeout <= 0.0 or self._vel_t is None:
            return False
        return (self._now() - self._vel_t) > self._vel_timeout

    def _tick(self):
        """タイマ本体。**例外を外へ出さない。**

        rclpy はタイマコールバックの例外を `spin` の外へ投げるので、1 サンプルの NaN や
        特異な解で**走行中にノードごと落ちる**。`main` の `finally` が detach するので
        機体は止まるが、**一発勝負の本番で run が終わる**のは割に合わない。
        ここでは 1 ティック分を捨てて 0 を出し、**続くようなら disarm する**。
        """
        try:
            self._tick_impl()
        except Exception as e:  # noqa: BLE001
            self._fail += 1
            self.get_logger().error(
                f"制御の計算に失敗 ({type(e).__name__}: {e})。この周期は 0 を出す "
                f"[{self._fail}/{FAIL_LIMIT}]", throttle_duration_sec=1.0)
            try:
                # **disarm 中は publish しない。** `_emit` は runnable=true を付けるので、
                # ここで出すと e-stop 中に毎周期 attach し直すことになる
                if self._arm.armed:
                    self._emit(np.zeros(8))
            except Exception:  # noqa: BLE001
                pass
            if self._fail >= FAIL_LIMIT:
                self.get_logger().error(
                    f"{FAIL_LIMIT} 周期続けて失敗した。**disarm する。** "
                    "原因を直すまで arm し直さないこと")
                self._arm.disarm("制御の計算が連続で失敗")
        else:
            # **disarm 中は「復帰した」と言わない。** 制御の経路を通らずに抜けているだけで、
            # 直った証拠が無い (壊れたまま disarm されている最中がまさにこれ)
            if self._arm.armed and self._fail:
                self.get_logger().warning(f"制御の計算が復帰した ({self._fail} 周期ぶん失敗)")
                self._fail = 0

    def _tick_impl(self):
        if not self._arm.armed:
            self._detach_all()
            return

        if self._imu_stale():
            # 姿勢の基準が無いまま推力を出さない。**出力は 0 にするが disarm はしない** —
            # 正浮力なので 0 出力は浮上側に外れる (沈むより安全)。IMU が戻れば自動で復帰
            why = ("IMU が 1 つも来ていない" if self._imu_t is None
                   else f"IMU が {self._now() - self._imu_t:.1f} s 途切れている")
            self.get_logger().error(
                f"{why} (imu_timeout={self._imu_timeout:.1f} s)。"
                "**姿勢制御を止めて出力を 0 にする。** 戻れば自動で再開する",
                throttle_duration_sec=2.0)
            # 観測器は**止めずに 0 指令で進める**。止めると断の前の速度を保持したまま
            # 復帰するので、初手で存在しない速度に制動をかける
            self._ctl.obs.update(np.zeros(8), self._quat, self._dt)
            self._emit(np.zeros(8))
            return

        from umiusi_perception.classical import cad_wrench_from_modes, rep103_from_cad

        ori_err = sub_quat(self._target, self._quat)
        if not self._hold_yaw:
            # yaw 成分を落とす = その軸まわりの姿勢誤差を 0 として扱う。rl_attitude_node と
            # 同じ扱い。**gyro は落とさない** — D 項は yaw の回転を減衰させ続けてよく、
            # 落とすと切った瞬間に回り始める
            ori_err = ori_err.copy()
            ori_err[YAW_IDX] = 0.0
        # 観測器には「直前に出した指令」を正規化したまま渡す (servo は ±1 = ±90 deg)。
        # レート制限後の実際の指令を使う — 制限前の値を渡すと、出していない指令で
        # 速度を積分することになる。
        # 観測器は CAD 系で返し、制御器は REP-103 を取る。変換は必ずこの関数で行う
        v_hat = self._ctl.obs.update(self._action, self._quat, self._dt)
        vel_cmd = self._vel_cmd
        if self._vel_stale():
            if not self._vel_dead:
                self._vel_dead = True
                self.get_logger().warning(
                    f"速度指令が {self._vel_timeout:.1f} s 来ない。**並進を 0 にする**"
                    " (姿勢目標は保持)。指令が再開すれば自動で戻る")
            vel_cmd = np.zeros(3)
        elif self._vel_dead:
            self._vel_dead = False
            self.get_logger().info("速度指令が戻った")
        modes = self._ctl.wrench(ori_err, self._gyro, vel_cmd,
                                 rep103_from_cad(v_hat), self._max_duty)
        # モード ([-1,1]) -> CAD 系のニュートン。cap は wrench が更新したフィルタ後の値
        wrench = cad_wrench_from_modes(modes, self._ctl.f_max_total(self._ctl.cap))
        action = self._alloc.allocate(wrench, self._max_duty)
        self._emit(action)

    def _emit(self, action):
        servo_target = np.asarray(action[:4], dtype=float) * 90.0
        duty_target = np.clip(np.asarray(action[4:], dtype=float),
                              -self._max_duty, self._max_duty)
        self._servo_cmd = slew(self._servo_cmd, servo_target, self._servo_slew, self._dt)
        self._duty_cmd = slew(self._duty_cmd, duty_target, self._thrust_slew, self._dt)
        self._action = np.concatenate([self._servo_cmd / 90.0, self._duty_cmd])
        if not self._publish:      # compute-only: 指令は出さないが観測器は回す
            return
        for k, p in enumerate(POSITIONS):
            out = ThrusterOutput()
            out.runnable = ThrusterRunnable(esc=True, servo=True)
            # 符号は**出口だけ**で掛ける。ここより上流 (観測器に渡す self._action、
            # アロケータ、スルーレート) はモデルの座標系のままにしておく — 途中で掛けると
            # 観測器が「出していない指令」で速度を積分する
            out.duty_cycle = float(self._duty_cmd[k]) * self._thrust_sign[k]
            # 単位は度 (known_issues B-13)。範囲外は CAN 送信が失敗するので ±90 に収める
            out.angle = max(-90.0, min(90.0,
                                       float(self._servo_cmd[k]) * self._servo_sign[k]))
            self._pubs[p].publish(out)

    def _detach_all(self):
        """DISARM: ゼロ出力 + runnable false。制御器の内部状態も戻す。

        `reset()` を呼ばないと、積分器・特異点回避の warm start・cap の LPF が
        次の arm に持ち越される (RL 側で同じ問題を踏んだ: 積分器がレールに張り付いたまま
        arm すると初手で max_duty のキックが出る)。
        """
        self._ctl.reset()
        self._alloc.reset()
        # **方位の積分器も戻す。** `cmd_target_yaw_mode="rate"` で積んだ目標方位を持ち越すと、
        # disarm 中に機体が流された分がそのまま誤差になり、再 arm の初手で大きな指令が出る。
        # 流された量が 180 度を超えていれば、そこが安定平衡になって出られなくなる
        # (2026-09-12 に踏んだ形)。次の arm で現在方位に取り直す
        self._yaw_sp = None
        self._yaw_t_prev = None
        # デッドマンも武装解除する。disarm 中に溜まった「来ていない時間」を持ち越すと、
        # 再 arm の初手で「速度指令が来ない」と誤検出する。
        # **同時に速度指令そのものも param の値へ戻す。** 時計だけ戻して指令を残すと、
        # 「指令が途切れた -> disarm -> 再 arm」で**保護が外れたまま古い速度で走り出す**
        self._vel_t, self._vel_dead = None, False
        self._vel_cmd = np.array([float(self.get_parameter("vel_cmd").value), 0.0, 0.0])
        # 失敗カウンタも戻す。持ち越すと再 arm 後の 1 回目の失敗で即 disarm になる
        self._fail = 0
        self._servo_cmd[:] = 0.0
        self._duty_cmd[:] = 0.0
        self._action[:] = 0.0
        if not self._publish:
            return
        for p in POSITIONS:
            out = ThrusterOutput()
            out.runnable = ThrusterRunnable(esc=False, servo=False)
            out.duty_cycle = 0.0
            out.angle = 0.0
            self._pubs[p].publish(out)


def main():
    rclpy.init()
    node = ClassicalAttitudeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._detach_all()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
