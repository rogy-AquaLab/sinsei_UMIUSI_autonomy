#!/usr/bin/env python3
"""thruster_cmd — 較正実験用のスラスタ直接指令 (issue #18 の実験 1/3/4/6/8 を 1 本で)。

`/cmd/direct/thruster_controller/output_*` に ThrusterOutput を 50 Hz で publish する。
RL もアロケータも通さない「素の指令」— 較正実験はこれで駆動する。**rl_attitude_node と
同時に動かさないこと** (同じトピックを取り合う)。

    python3 thruster_cmd.py spin                     # 実験 1: 1 基ずつ、回したまま servo を振る
    python3 thruster_cmd.py step --ch lf --angle 80  # 実験 3: サーボステップ 0→80°→0 ×3
    python3 thruster_cmd.py sweep --ch lf            # 実験 4: 推力ベンチ duty ±0.2..±1.0
    python3 thruster_cmd.py pose                     # 全基 servo 0 deg / duty 0.1 で保持 (向きの確認)
    python3 thruster_cmd.py pose --angle 45          # 全基 45 deg に寝かせて保持
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
    """回しっぱなしのままサーボを振る。推力の前後とサーボの上下を同じ映像で見比べられる。"""
    targets = [p for p in POSITIONS if p in a.ch] if a.ch else list(POSITIONS)
    confirm(f"実験 1: {', '.join(targets)} を 1 基ずつ duty {a.duty} で回し、"
            f"**回したまま** servo 0 → +{a.angle:.0f}° → 0 → -{a.angle:.0f}° → 0 を各 {a.seconds} s。"
            "推力の向き (前/後) とサーボの傾く向き (上/下) を同時に目視/動画で記録",
            a.yes)
    for p in targets:
        input(f"[{p}] Enter で回します: ") if not a.yes else None
        for ang, what in ((0.0, "水平 — 推力の前後を見る"),
                          (a.angle, f"servo +{a.angle:.0f}° — 上下どちらに傾くか"),
                          (0.0, "水平"),
                          (-a.angle, f"servo -{a.angle:.0f}°"),
                          (0.0, "水平")):
            drv.hold(a.seconds, duty={p: a.duty}, angle={p: ang},
                     label=f"{p}: duty {a.duty} / {what}")
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
    # **符号と挙動の対応は 2026-09-13 にバンドルの幾何で計算し直した。以前は逆だった。**
    # 4 基は接線方向に付いた偶力配置なので:
    #   全基同符号        -> 合力ちょうど 0 の**純粋な旋回** (上から見て右回り)
    #   lf+ lb+ rb- rf-  -> 合力が前方のみ (yaw ほぼ 0) の**前進**
    # 以前はこの 2 つのラベルが入れ替わっていて、「前進」と表示して旋回していた。
    sign = dict.fromkeys(POSITIONS, 1) if a.yaw else {"lf": 1, "lb": 1, "rb": -1, "rf": -1}
    what = "旋回 (全基同符号・合力 0 の偶力)" if a.yaw else "前進 (lf+ lb+ rb- rf-)"
    confirm(f"実験 6: {what} duty {a.duty} を {a.seconds} s。"
            "プール長辺方向・中央から。前進と後退 (--duty 負) の両方取ること", a.yes)
    drv.hold(a.seconds, duty={p: sign[p] * a.duty for p in POSITIONS},
             label=f"{what} duty {a.duty}")


def _expect_words(angle_deg, duty, chans):
    """バンドルの幾何から「この指令で何が起きるはずか」を日本語にする。

    地上/水中を問わず、**指令を出す前に何を見ればよいか**が分からないと確認にならない。
    バンドルが読めなければ黙って諦める (このツールは単体で動くのが取り柄なので、
    バンドルを必須にはしない)。
    """
    try:
        import json
        from pathlib import Path as _P
        from ament_index_python.packages import get_package_share_directory
        c = json.loads((_P(get_package_share_directory("umiusi_autonomy"))
                        / "config" / "classical_bundle.json").read_text())["contract"]
        ax = np.asarray(c["thrust_axes"], float)
        pv = np.asarray(c["pivots_from_com"], float)
    except Exception:                                    # noqa: BLE001
        return None
    y_up = np.array([0.0, 1.0, 0.0])
    phi = math.radians(angle_deg)
    f = np.zeros(3)
    t = np.zeros(3)
    for k, p in enumerate(POSITIONS):
        if p not in chans:
            continue
        v = (math.cos(phi) * ax[k] + math.sin(phi) * y_up) * duty
        f += v
        t += np.cross(pv[k], v)
    # CAD: +X 前 / +Y 上 / +Z 右舷。トルクの +Y まわりが yaw
    parts = []
    if abs(f[0]) > 1e-3:
        parts.append("前進" if f[0] > 0 else "後退")
    if abs(f[2]) > 1e-3:
        parts.append("右舷へ横移動" if f[2] > 0 else "左舷へ横移動")
    if abs(f[1]) > 1e-3:
        parts.append("上昇" if f[1] > 0 else "下降")
    if abs(t[1]) > 1e-3:
        parts.append("上から見て左回り" if t[1] > 0 else "上から見て右回り")
    if abs(t[0]) > 1e-3:
        parts.append("左舷が下がる" if t[0] > 0 else "右舷が下がる")
    if abs(t[2]) > 1e-3:
        parts.append("機首が上がる" if t[2] > 0 else "機首が下がる")
    return "・".join(parts) if parts else "力もモーメントも出ない (釣り合い)"


def cmd_pose(drv, a):
    """**全基を規定の姿勢に置いて保持する。** 方向と向きを一度に目で見るためのもの。

    1 基ずつ見る `spin` / `thrust_sign_check.py --ground` と違い、**4 基の相対関係**が
    見える: サーボが全部同じ向きに寝ているか、噴流が揃っているか、1 基だけ違わないか。
    姿勢制御を入れる前に、ここで食い違いを潰しておく。
    """
    chans = set(a.ch) if a.ch else set(POSITIONS)
    exp = _expect_words(a.angle, a.duty, chans)
    print(f"規定姿勢: サーボ {a.angle:+.0f}° / duty {a.duty:+.2f} / 対象 {sorted(chans)}")
    if exp:
        print(f"  **この指令で起きるはず**: {exp}")
        print("  (地上なら噴流の向きで、水中なら機体の動きで確かめる)")
    else:
        print("  (バンドルが読めないので期待値は出せない)")
    confirm(f"{a.seconds:.0f} s 保持する。**水中なら機体が動く。** 地上なら空回しなので"
            "長く回さないこと", a.yes)
    drv.hold(a.seconds,
             duty={p: (a.duty if p in chans else 0.0) for p in POSITIONS},
             angle={p: (a.angle if p in chans else 0.0) for p in POSITIONS},
             label=f"servo {a.angle:+.0f}° duty {a.duty:+.2f}")


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
    s.add_argument("--seconds", type=float, default=8.0, help="各ステップの保持時間 [s]")
    s.add_argument("--angle", type=float, default=45.0, help="サーボの振り角 [deg]")
    s.add_argument("--ch", nargs="+", choices=POSITIONS, default=None,
                   help="対象 ch (既定は 4 基すべて)")

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

    s = sub.add_parser("pose", help="全基を規定の姿勢 (サーボ角 + duty) に置いて保持する")
    s.add_argument("--angle", type=float, default=0.0, help="サーボ角 [deg]。既定 0 = 水平")
    s.add_argument("--duty", type=float, default=0.1, help="duty (負で逆)。既定 0.1")
    s.add_argument("--seconds", type=float, default=20.0)
    s.add_argument("--ch", nargs="+", choices=POSITIONS, default=None,
                   help="対象の基 (既定は全 4 基)")

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
        {"spin": cmd_spin, "step": cmd_step, "sweep": cmd_sweep, "pose": cmd_pose,
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
