#!/usr/bin/env python3
"""thruster_cmd — 較正実験用のスラスタ直接指令 (issue #18 の実験 1/3/4/6/8 を 1 本で)。

`/cmd/direct/thruster_controller/output_*` に ThrusterOutput を 50 Hz で publish する。
RL もアロケータも通さない「素の指令」— 較正実験はこれで駆動する。**rl_attitude_node と
同時に動かさないこと** (同じトピックを取り合う)。

    python3 thruster_cmd.py spin                     # 実験 1: 1 基ずつ duty 0.2 / servo +45°
    python3 thruster_cmd.py step --ch lf --angle 80  # 実験 3: サーボステップ 0→80°→0 ×3
    python3 thruster_cmd.py sweep --ch lf            # 実験 4: 推力ベンチ duty ±0.2..±1.0
    python3 thruster_cmd.py steady --duty 0.3        # 実験 6: 全基前進 10 s (--yaw で旋回)
    python3 thruster_cmd.py excite --seconds 120     # 実験 8: 有界ランダム励起 (world model 用)

安全:
  * Ctrl-C / 終了時は必ず**ゼロ出力 + detach** (runnable false) を送ってから抜ける
  * duty は既定 0.4 まで。それ以上 (推力ベンチの ±1.0) は `--allow-full` を明示
  * 開始前に実行内容を表示して Enter 待ち (`--yes` でスキップ)
  * excite は duty 上限・サーボレンジを絞った滑らかなランダムウォーク (毎ステップ独立の
    乱数だとスラスタに厳しいだけで励起にならない)
"""
from __future__ import annotations

import argparse
import math
import sys
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sinsei_umiusi_msgs.msg import ThrusterOutput, ThrusterRunnable

POSITIONS = ("lf", "lb", "rb", "rf")
CMD_PREFIX = "/cmd/direct/thruster_controller/output_"
HZ = 50.0


class Driver(Node):
    def __init__(self):
        super().__init__("thruster_cmd")
        self.pubs = {p: self.create_publisher(ThrusterOutput, CMD_PREFIX + p, 10) for p in POSITIONS}

    def send(self, duty=None, angle=None):
        """duty/angle: dict {pos: value} または全 ch 同値のスカラ。None の ch はゼロ出力。"""
        for p in POSITIONS:
            d = (duty.get(p, 0.0) if isinstance(duty, dict) else duty) or 0.0
            a = (angle.get(p, 0.0) if isinstance(angle, dict) else angle) or 0.0
            out = ThrusterOutput()
            out.runnable = ThrusterRunnable(esc=True, servo=True)
            out.duty_cycle = float(d)
            out.angle = float(a)                      # degrees (rl_attitude_node と同じ規約)
            self.pubs[p].publish(out)

    def detach(self):
        for p in POSITIONS:
            out = ThrusterOutput()
            out.runnable = ThrusterRunnable(esc=False, servo=False)
            self.pubs[p].publish(out)

    def hold(self, seconds, duty=None, angle=None, label=""):
        """指令を 50 Hz で seconds 秒送り続ける。"""
        if label:
            print(f"  {label} ({seconds:.1f} s)")
        end = time.time() + seconds
        while time.time() < end:
            self.send(duty, angle)
            time.sleep(1.0 / HZ)


def confirm(text, yes):
    print(text)
    if not yes:
        input("Enter で開始 (Ctrl-C で中止): ")


def cmd_spin(drv, a):
    confirm(f"実験 1: {', '.join(POSITIONS)} を 1 基ずつ duty {a.duty} で {a.seconds} s 回し、"
            f"続けて servo +45° を保持します。物理位置と回転方向・サーボ向きを目視/動画で記録",
            a.yes)
    for p in POSITIONS:
        input(f"[{p}] Enter で回します: ") if not a.yes else None
        drv.hold(a.seconds, duty={p: a.duty}, label=f"{p}: duty {a.duty}")
        drv.hold(1.0, label=f"{p}: 停止")
        drv.hold(a.seconds, angle={p: 45.0}, label=f"{p}: servo +45 deg")
        drv.hold(1.0, label=f"{p}: 停止")


def cmd_step(drv, a):
    confirm(f"実験 3: {a.ch} のサーボを 0→{a.angle}°→0 のステップ ×{a.repeat} 回。"
            "スマホ slow-mo で撮影しておくこと", a.yes)
    for i in range(a.repeat):
        drv.hold(2.0, label=f"#{i + 1} 0 deg")
        drv.hold(2.0, angle={a.ch: float(a.angle)}, label=f"#{i + 1} {a.angle} deg")
    drv.hold(2.0, label="0 deg")


def cmd_sweep(drv, a):
    duties = [x for m in a.points for x in (m, -m)]
    peak = max(abs(d) for d in duties)
    if peak > 0.4 and not a.allow_full:
        sys.exit(f"duty {peak} > 0.4 には --allow-full が必要です (推力ベンチで機体を固定してから)")
    confirm(f"実験 4: {a.ch} を duty {duties} で各 {a.dwell} s 駆動 (間に {a.rest} s 停止)。"
            "秤の読みを各 dwell ごとに記録", a.yes)
    for d in duties:
        drv.hold(a.dwell, duty={a.ch: d}, label=f"duty {d:+.1f}")
        drv.hold(a.rest, label="停止 (秤ゼロ確認)")


def cmd_steady(drv, a):
    sign = {"lf": 1, "lb": 1, "rb": -1, "rf": -1} if a.yaw else dict.fromkeys(POSITIONS, 1)
    what = "旋回 (左右逆転)" if a.yaw else "前進 (全基同符号)"
    confirm(f"実験 6: {what} duty {a.duty} を {a.seconds} s。"
            "プール長辺方向・中央から。前進と後退 (--duty 負) の両方取ること", a.yes)
    drv.hold(a.seconds, duty={p: sign[p] * a.duty for p in POSITIONS},
             label=f"{what} duty {a.duty}")


def cmd_excite(drv, a):
    rng = np.random.default_rng(a.seed)
    confirm(f"実験 8: 有界ランダム励起 {a.seconds} s (duty ≤ {a.duty_max}, servo ±{a.servo_max}°, "
            f"seed {a.seed})。中央スタート・テザー係必須。bag は --profile teleop で検品", a.yes)
    duty = np.zeros(4)
    angle = np.zeros(4)
    t_end = time.time() + a.seconds
    step = 0
    while time.time() < t_end:
        if step % int(HZ * a.hold_s) == 0:           # hold_s ごとに新しい目標へ滑らかに向かう
            duty_t = rng.uniform(-a.duty_max, a.duty_max, 4)
            angle_t = rng.uniform(-a.servo_max, a.servo_max, 4)
        duty += np.clip(duty_t - duty, -a.duty_slew / HZ, a.duty_slew / HZ)
        angle += np.clip(angle_t - angle, -a.servo_slew / HZ, a.servo_slew / HZ)
        drv.send(dict(zip(POSITIONS, duty)), dict(zip(POSITIONS, angle)))
        time.sleep(1.0 / HZ)
        step += 1
    print("励起終了")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--yes", action="store_true", help="確認プロンプトを省略")
    sub = ap.add_subparsers(dest="cmd", required=True)

    s = sub.add_parser("spin", help="実験 1: スラスタ ID (1 基ずつ回す)")
    s.add_argument("--duty", type=float, default=0.2)
    s.add_argument("--seconds", type=float, default=3.0)

    s = sub.add_parser("step", help="実験 3: サーボステップ応答")
    s.add_argument("--ch", choices=POSITIONS, required=True)
    s.add_argument("--angle", type=float, default=80.0, help="ステップ量 [deg] (80 と 10 の両方取る)")
    s.add_argument("--repeat", type=int, default=3)

    s = sub.add_parser("sweep", help="実験 4: 推力ベンチの duty 階段")
    s.add_argument("--ch", choices=POSITIONS, required=True)
    s.add_argument("--points", type=float, nargs="+", default=[0.2, 0.4, 0.6, 0.8, 1.0])
    s.add_argument("--dwell", type=float, default=5.0)
    s.add_argument("--rest", type=float, default=3.0)
    s.add_argument("--allow-full", action="store_true", help="duty > 0.4 を許可 (要・機体固定)")

    s = sub.add_parser("steady", help="実験 6: 定常前進/旋回")
    s.add_argument("--duty", type=float, default=0.3, help="負で後退")
    s.add_argument("--seconds", type=float, default=10.0)
    s.add_argument("--yaw", action="store_true", help="左右逆転で旋回")

    s = sub.add_parser("excite", help="実験 8: 有界ランダム励起 (world model データ)")
    s.add_argument("--seconds", type=float, default=120.0)
    s.add_argument("--duty-max", type=float, default=0.3)
    s.add_argument("--servo-max", type=float, default=80.0)
    s.add_argument("--hold-s", type=float, default=1.5, help="目標を引き直す間隔 [s]")
    s.add_argument("--duty-slew", type=float, default=1.0, help="duty の変化率上限 [/s]")
    s.add_argument("--servo-slew", type=float, default=200.0, help="サーボ目標の変化率上限 [deg/s]")
    s.add_argument("--seed", type=int, default=0)

    a = ap.parse_args()
    if a.cmd in ("spin", "steady") and abs(getattr(a, "duty", 0.0)) > 0.4:
        sys.exit("duty > 0.4 は spin/steady では使いません (推力ベンチは sweep --allow-full)")

    rclpy.init()
    drv = Driver()
    time.sleep(0.5)          # publisher のマッチング待ち
    try:
        {"spin": cmd_spin, "step": cmd_step, "sweep": cmd_sweep,
         "steady": cmd_steady, "excite": cmd_excite}[a.cmd](drv, a)
    except KeyboardInterrupt:
        print("\n中断")
    finally:
        for _ in range(5):   # 確実にゼロ + detach
            drv.send()
            time.sleep(0.02)
        drv.detach()
        drv.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
