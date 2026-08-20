#!/usr/bin/env python3
"""機体を手で動かしながら IMU サニティフィルタの挙動を確かめる。

静止時の健全性は `imu_glitch.py`。こちらは **動かしている最中に正常なサンプルまで
弾いていないか**を見るためのもの。`rl_attitude_node` と同じ `ImuSanity` を同じ QoS で
通し、閾値 (`imu_max_gyro` / `imu_max_step_deg`) に対する余裕を数値で出す。

    python3 tools/imu_sanity_check.py               # 60 秒。その間に機体を手で振る
    python3 tools/imu_sanity_check.py --duration 90
    python3 tools/imu_sanity_check.py --max-gyro 10 --max-step-deg 30   # 閾値を変えて試す
    python3 tools/imu_sanity_check.py --save /tmp/imu_raw.csv           # 生データを残す

`--save` した CSV は `tools/imu_sanity_replay.py` に食わせると、**手元の PC で**閾値を
振り直して何度でも評価できる (ROS も実機も要らない)。実機で測り直すより速い。

**姿勢の跳躍は「最後に *採用* されたサンプル」との差分**で判定される。棄却が続くと
比較対象が古いままになるので、速く動かすほど連鎖しやすい。連続棄却が `stale_after`
(既定 5) を超えると `rl_attitude_node` は値が古いと判断する — その回数も報告する。
"""
from __future__ import annotations

import argparse
import math
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu

from umiusi_rl_control.imu_sanity import ImuSanity, _angle_between

MOVING_GYRO = 0.15   # これを超えたら「動かしている」とみなす [rad/s] (≒ 8.6 deg/s)


def pct(xs, p):
    if not xs:
        return 0.0
    s = sorted(xs)
    return s[min(len(s) - 1, int(len(s) * p / 100.0))]


class Checker(Node):
    def __init__(self, a):
        super().__init__("imu_sanity_check")
        # rl_attitude_node と同じ条件で購読する (RELIABLE / depth 1)。ここを変えると
        # 取りこぼしで「サンプル間の姿勢変化」が実際より大きく出て再現にならない。
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Imu, a.topic, self._on_imu, qos)
        self.san = ImuSanity(max_gyro=a.max_gyro, max_step_deg=a.max_step_deg)
        self.a = a
        self.t0 = None
        self.n = 0
        self.gyro_all: list[float] = []
        self.gyro_moving: list[float] = []
        self.step_all: list[float] = []
        self.step_moving: list[float] = []
        self.reasons: dict[str, int] = {}
        self.rejected_detail: list[str] = []
        self.consec = 0
        self.consec_max = 0
        self.stale_events = 0
        self._last_print = 0.0
        self._win_gyro = 0.0
        self._win_step = 0.0
        # 生データ。閾値の評価は後から手元でやり直せるようにする (imu_sanity_replay.py)。
        # 50 Hz × 数分でも数千行なので、書き出しは最後にまとめて行う
        self.raw: list[tuple] = []
        print(f"'{a.topic}' を {a.duration:.0f} 秒みます。"
              f"閾値 gyro {a.max_gyro} rad/s / step {a.max_step_deg} deg")
        print("**機体を手で動かしてください** (ゆっくり → 速く、傾け・ヨー振り)\n")

    def _on_imu(self, msg):
        now = time.time()
        if self.t0 is None:
            self.t0 = now
        self.n += 1
        q, g = msg.orientation, msg.angular_velocity
        gmax = max(abs(g.x), abs(g.y), abs(g.z))

        # 採用済みの直前値との角度差。ImuSanity が内部で見ているものと同じ
        step_deg = 0.0
        prev = self.san.last
        if prev is not None:
            n = math.sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z)
            if n > 1e-9:
                step_deg = math.degrees(
                    _angle_between(prev.quat, (q.w / n, q.x / n, q.y / n, q.z / n)))

        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.raw.append((now - self.t0, stamp, q.w, q.x, q.y, q.z, g.x, g.y, g.z))

        _, reason = self.san.update((q.w, q.x, q.y, q.z), (g.x, g.y, g.z))

        moving = gmax > MOVING_GYRO
        self.gyro_all.append(gmax)
        self.step_all.append(step_deg)
        if moving:
            self.gyro_moving.append(gmax)
            self.step_moving.append(step_deg)

        if reason is not None:
            key = reason.split(" (")[0]
            self.reasons[key] = self.reasons.get(key, 0) + 1
            self.consec += 1
            self.consec_max = max(self.consec_max, self.consec)
            if self.consec == self.san.stale_after + 1:
                self.stale_events += 1
            if len(self.rejected_detail) < 20:
                self.rejected_detail.append(
                    f"t={now - self.t0:6.2f}s  gyro={gmax:6.2f}  step={step_deg:6.1f}deg  {reason}")
        else:
            self.consec = 0

        self._win_gyro = max(self._win_gyro, gmax)
        self._win_step = max(self._win_step, step_deg)
        if now - self._last_print >= 1.0:
            self._last_print = now
            print(f"\r  {now - self.t0:5.1f}s  {self.n:6d} サンプル  棄却 {self.san.rejected:4d}"
                  f" ({self.san.reject_ratio:5.2%})  直近1秒: gyro {self._win_gyro:5.2f} rad/s"
                  f" / step {self._win_step:5.1f} deg   ", end="", flush=True)
            self._win_gyro = self._win_step = 0.0

        if now - self.t0 >= self.a.duration:
            raise SystemExit(0)

    def save(self):
        if not self.a.save or not self.raw:
            return
        with open(self.a.save, "w") as f:
            f.write("t_rel,stamp,qw,qx,qy,qz,gx,gy,gz\n")
            for r in self.raw:
                f.write("%.6f,%.6f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f\n" % r)
        print(f"  生データ {len(self.raw)} 行を保存: {self.a.save}")
        print("  手元で評価し直す: python3 tools/imu_sanity_replay.py <このファイル> --sweep")

    def report(self):
        a = self.a
        el = (time.time() - self.t0) if self.t0 else 0.0
        print("\n")
        print("=" * 68)
        if self.n == 0:
            print("  サンプルが 1 つも届きませんでした (control スタックは起動していますか?)")
            return 1
        print(f"  {self.n} サンプル / {el:.1f} 秒 ({self.n / el:.1f} Hz)")
        nm = len(self.gyro_moving)
        print(f"  うち動作中 (|gyro| > {MOVING_GYRO} rad/s): {nm} サンプル ({nm / self.n:.1%})")
        print()
        print(f"  棄却 {self.san.rejected} 件 ({self.san.reject_ratio:.3%})"
              f" / 連続棄却の最大 {self.consec_max} 回"
              f" / stale ({self.san.stale_after} 連続超え) {self.stale_events} 回")
        for k, v in sorted(self.reasons.items(), key=lambda kv: -kv[1]):
            print(f"    - {k}: {v} 件")
        if self.rejected_detail:
            print("  棄却されたサンプル (先頭 20 件):")
            for line in self.rejected_detail:
                print(f"    {line}")
        print()
        print(f"  {'':22} {'p50':>8} {'p95':>8} {'p99':>8} {'max':>8}   閾値      余裕")
        for label, xs, thr in (
            ("角速度 [rad/s] 全体", self.gyro_all, a.max_gyro),
            ("角速度 [rad/s] 動作中", self.gyro_moving, a.max_gyro),
            ("姿勢変化 [deg] 全体", self.step_all, a.max_step_deg),
            ("姿勢変化 [deg] 動作中", self.step_moving, a.max_step_deg),
        ):
            if not xs:
                continue
            mx = max(xs)
            margin = (thr / mx) if mx > 1e-9 else float("inf")
            print(f"  {label:22} {pct(xs, 50):8.2f} {pct(xs, 95):8.2f} {pct(xs, 99):8.2f}"
                  f" {mx:8.2f}   {thr:6.1f}   {margin:5.1f} 倍")
        print()
        ng = 0
        if self.stale_events > 0:
            print("  ⚠ 連続棄却が stale 判定に達しました。閾値が厳しすぎます"); ng += 1
        elif self.san.reject_ratio > 0.01:
            print("  ⚠ 棄却率が 1% を超えています。閾値の見直しを検討してください"); ng += 1
        if nm < 50:
            print("  ⚠ 動作中のサンプルがほとんどありません。機体を動かして測り直してください"); ng += 1
        if ng == 0:
            print("  問題なし — 動作中も正常サンプルは弾かれていません")
        print("=" * 68)
        return ng


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--topic", default="/state/imu")
    ap.add_argument("--duration", type=float, default=60.0, help="計測秒数")
    ap.add_argument("--max-gyro", type=float, default=10.0, help="imu_max_gyro と同じ値 [rad/s]")
    ap.add_argument("--max-step-deg", type=float, default=30.0, help="imu_max_step_deg と同じ値")
    ap.add_argument("--save", default="", help="生データ (CSV) の保存先。閾値の評価をやり直せる")
    a = ap.parse_args()

    rclpy.init()
    node = Checker(a)
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        rc = node.report()
        node.save()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    raise SystemExit(0 if rc == 0 else 1)


if __name__ == "__main__":
    main()
