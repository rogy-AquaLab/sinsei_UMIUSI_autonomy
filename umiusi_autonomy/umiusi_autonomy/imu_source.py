"""ImuSource — IMU の購読・軸解釈・サニティ・断検出を 1 箇所にまとめる。

``navigator_node`` と ``auto_target_generator`` は同じ IMU の扱いを別々に実装していた
(``_AXIS`` / ``yaw_rate_axis`` / ``yaw_rate_sign`` / ``ImuSanity`` / 断検出)。**同じものが
2 箇所にあると片方だけ直る** — 実際 ``yaw_rate_axis`` の不正値フォールバックは
``navigator_node`` だけ z に直され、``auto_target_generator`` は y のまま残っていた
(issue #19-5)。ここに寄せて、両ノードは「ヨーレートを読む」だけにする。

センサ解釈と運用はアクチュエータ層ではないので control には出せない。**autonomy 側で
共有するのが正しい持ち主**。``umiusi_rl_control.rl_attitude_node`` も ``ImuSanity`` を
使っているが、あちらは姿勢クォータニオンごと必要で形が違ううえ、パッケージを跨ぐと
依存の向きが逆転する (#19-5 の「中立なパッケージへ移す」が済むまで別実装のまま)。

Parameters (使う側のノードに宣言される)
---------------------------------------
imu_topic          : 購読するトピック (default /state/imu)。
yaw_rate_axis      : ヨーレートを載せている IMU 軸 (default "z", REP-103)。
                     **不正な軸名は z にフォールバックする** — 1 文字の typo で無言の
                     誤軸になるのを避けるため。
yaw_rate_sign      : 上記の符号 (default 1.0)。
imu_max_gyro       : サニティ: 角速度の閾値 [rad/s] (default 10.0, 0 以下で無効)。
imu_max_step_deg   : サニティ: 1 サンプルの姿勢跳躍上限 [deg] (default 30.0)。
imu_sanity_enforce : true で化けサンプルを破棄する (default false = 検出のみ)。
imu_timeout        : この秒数来なければ「断」とみなす (default 1.0, 0 以下で無効)。
"""

from __future__ import annotations

from umiusi_rl_control.imu_sanity import ImuSanity

_AXIS = {"x": 0, "y": 1, "z": 2}
_DEFAULT_AXIS = _AXIS["z"]        # REP-103 (x-fwd / y-left / z-up) のヨー軸


class ImuSource:
    """1 ノードぶんの IMU 購読。パラメータ宣言と購読の生成を分けてあるので、
    lifecycle ノード (``on_configure`` で購読を作る) からも使える。"""

    def __init__(self, node, default_topic: str = "/state/imu") -> None:
        self._node = node
        node.declare_parameter("imu_topic", default_topic)
        node.declare_parameter("yaw_rate_axis", "z")
        node.declare_parameter("yaw_rate_sign", 1.0)
        # IMU のサニティフィルタ (実機の化けサンプル対策)。0 以下で無効化できる。
        node.declare_parameter("imu_max_gyro", 10.0)
        node.declare_parameter("imu_max_step_deg", 30.0)
        # 既定は「検出するが破棄しない」。理由は imu_sanity.py 冒頭。
        node.declare_parameter("imu_sanity_enforce", False)
        # IMU が途切れたことに気付けるようにする。**ヨーレートは直近値を保持する**ので、
        # 断が起きると「回っているつもり」のまま制御が進む。8/25 の水中 run では autonomy
        # 区間だけで 15.44 s + 11.10 s の欠落があり (残り 800 s は 0.5 s 超の欠落ゼロ)、
        # コンソールにも bag にも痕跡が無かった。
        node.declare_parameter("imu_timeout", 1.0)

        self.topic = str(node.get_parameter("imu_topic").value)
        self._axis = _AXIS.get(
            str(node.get_parameter("yaw_rate_axis").value).lower(), _DEFAULT_AXIS)
        self._sign = float(node.get_parameter("yaw_rate_sign").value)
        self._sanity = ImuSanity(
            max_gyro=float(node.get_parameter("imu_max_gyro").value),
            max_step_deg=float(node.get_parameter("imu_max_step_deg").value),
            enforce=bool(node.get_parameter("imu_sanity_enforce").value))
        self._timeout = float(node.get_parameter("imu_timeout").value)

        self.yaw_rate = 0.0        # 直近の (符号・軸を当てた) ヨーレート [rad/s]
        self._last_t = None        # None = まだ 1 つも来ていない
        self._sub = None

    # ---- 購読の生成 / 破棄 (lifecycle ノード用に分けてある) ----
    def create_subscription(self, depth: int = 10):
        from sensor_msgs.msg import Imu       # ノードの起動パスから外す
        self._sub = self._node.create_subscription(Imu, self.topic, self._on_imu, depth)
        return self._sub

    def destroy(self) -> None:
        if self._sub is not None:
            self._node.destroy_subscription(self._sub)
            self._sub = None

    # ---- 受信 ----
    def _on_imu(self, msg) -> None:
        self._last_t = self._now()
        # 実機の BNO055 は物理的にありえないサンプルを混ぜてくる (ゼロクォータニオン、
        # 角速度の int16 フルスケール張り付き、姿勢の跳躍)。ヨーレートをそのまま制御に
        # 使うので、1 発のスパイクで制御が跳ねる。ただし **既定では検出するだけで弾かない**
        # (`imu_sanity_enforce`)。理由は imu_sanity.py 冒頭。
        q, g = msg.orientation, msg.angular_velocity
        sample, reason = self._sanity.update((q.w, q.x, q.y, q.z), (g.x, g.y, g.z))
        if reason is not None:
            self._node.get_logger().warning(
                self._sanity.describe(reason), throttle_duration_sec=5.0)
            if sample is None:
                return          # まだ 1 つも有効値が無い
        # sensor_msgs/Imu.angular_velocity is RAD/S (ROS standard), which is what the FSM wants.
        self.yaw_rate = self._sign * sample.gyro[self._axis]

    def _now(self) -> float:
        return self._node.get_clock().now().nanoseconds * 1e-9

    # ---- 断検出 ----
    def stale_for(self) -> float:
        """IMU が何秒途切れているか。0.0 = 生きている / -1.0 = まだ 1 つも来ていない。

        検出と警告だけを行う — **ヨーレートは直近値のまま**にする。ゼロにすると FSM の
        探索 (``_swept += |yaw_rate|·dt``) が進まなくなり、その場旋回から抜けられない。
        探索の打ち切り自体は FSM (``umiusi_perception``) の責務。
        """
        if self._timeout <= 0.0:
            return 0.0
        if self._last_t is None:
            return -1.0
        gap = self._now() - self._last_t
        return gap if gap > self._timeout else 0.0

    def warn_if_stale(self, throttle_sec: float = 2.0) -> bool:
        """断なら警告を出して True を返す。制御は止めない (上記 ``stale_for`` の理由)。"""
        stale = self.stale_for()
        if stale == 0.0:
            return False
        self._node.get_logger().warning(
            ("IMU が 1 つも来ていません" if stale < 0.0 else f"IMU が {stale:.1f} s 途切れています")
            + f" — ヨーレートは直近値 ({self.yaw_rate:+.3f} rad/s) を保持したまま制御しています",
            throttle_duration_sec=throttle_sec)
        return True
