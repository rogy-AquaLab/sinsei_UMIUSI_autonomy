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

from umiusi_common.arm import ArmState
from umiusi_common.imu_sanity import ImuSanity
from umiusi_rl_control_msgs.msg import AttitudeTarget

POSITIONS = ("lf", "lb", "rb", "rf")
CMD_PREFIX = "/cmd/direct/thruster_controller/output_"
CURRENT_SETPOINT_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


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
        # `/cmd/direct` は control の max_duty もスルーレート制限も素通りする
        # (known_issues B-12) ので、ここで掛けなければ誰も掛けない (known_issues A-11)。
        # 古典制御の特異点回避はこのサーボ速度限界を前提に調整してある
        self.declare_parameter("servo_slew_deg_per_s", 250.0)
        self.declare_parameter("thrust_slew_per_s", 4.0)
        self.declare_parameter("servo_sign", [1.0, 1.0, 1.0, 1.0])
        self.declare_parameter("imu_max_gyro", 10.0)
        self.declare_parameter("imu_max_step_deg", 30.0)
        self.declare_parameter("imu_sanity_enforce", False)

        self._dt = 1.0 / float(self.get_parameter("control_hz").value)
        self._max_duty = abs(float(self.get_parameter("max_duty").value))
        self._publish = bool(self.get_parameter("publish").value)
        self._servo_slew = float(self.get_parameter("servo_slew_deg_per_s").value)
        self._thrust_slew = float(self.get_parameter("thrust_slew_per_s").value)
        self._servo_sign = [float(v) for v in self.get_parameter("servo_sign").value]

        if not self._load(self.get_parameter("bundle_path").value):
            raise SystemExit(1)

        self._sanity = ImuSanity(
            max_gyro=float(self.get_parameter("imu_max_gyro").value),
            max_step_deg=float(self.get_parameter("imu_max_step_deg").value),
            enforce=bool(self.get_parameter("imu_sanity_enforce").value))

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

        self._arm = ArmState(self, self._detach_all,
                             start_armed=bool(self.get_parameter("start_armed").value))
        self.create_timer(self._dt, self._tick)
        self.get_logger().info(
            f"classical attitude: {self._dt * 1000:.0f} ms, max_duty={self._max_duty:.2f}, "
            f"publish={self._publish}")

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
        self._ctl = ClassicalController(plant, **b.get("gains", {}))
        self._alloc = GeneralAllocator(plant, **b.get("allocator", {}))
        # 較正前のバンドルで絶対値を信じないこと。相対比較には使える
        for k in b.get("uncalibrated", []):
            self.get_logger().warn(f"未較正の契約値: {k}")
        self.get_logger().info(
            f"bundle: {path} (cap_ref={plant.cap_ref:.2f}, "
            f"{plant.control_rate_hz:.0f} Hz, 到達速度 @max_duty="
            f"{self._ctl.reachable_speed(self._max_duty):.3f} m/s)")
        return True

    def _on_imu(self, msg):
        q = msg.orientation
        g = msg.angular_velocity
        sample, _reason = self._sanity.update((q.w, q.x, q.y, q.z), (g.x, g.y, g.z))
        if sample is None:
            return
        self._quat = np.asarray(sample.quat, dtype=float)
        self._gyro = np.asarray(sample.gyro, dtype=float)

    def _on_setpoint(self, msg):
        o = msg.orientation
        self._target = np.array([o.w, o.x, o.y, o.z], dtype=float)
        v = msg.velocity
        self._vel_cmd = np.array([v.x, v.y, v.z], dtype=float)

    def _tick(self):
        if not self._arm.armed:
            self._detach_all()
            return

        from umiusi_perception.classical import cad_wrench_from_modes, rep103_from_cad

        ori_err = sub_quat(self._target, self._quat)
        # 観測器には「直前に出した指令」を正規化したまま渡す (servo は ±1 = ±90 deg)。
        # レート制限後の実際の指令を使う — 制限前の値を渡すと、出していない指令で
        # 速度を積分することになる。
        # 観測器は CAD 系で返し、制御器は REP-103 を取る。変換は必ずこの関数で行う
        v_hat = self._ctl.obs.update(self._action, self._quat, self._dt)
        modes = self._ctl.wrench(ori_err, self._gyro, self._vel_cmd,
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
            out.duty_cycle = float(self._duty_cmd[k])
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
