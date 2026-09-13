#!/usr/bin/env python3
"""1 基ずつ回して、推力の向きがモデルと合っているかを**その場で**判定する。

2026-09-12 のプール bag で、指令したモーメントと実測の角加速度が 3 軸とも逆相関していた
(corr yaw −0.77〜−0.87)。ただしあの run は姿勢制御が 4 基を**一斉に**動かしていたので
(duty の相互相関 1.000)、どの基が反転しているかは原理的に分けられない。**1 基ずつ励起
すれば分けられる。** それをやるのがこれ。

    python3 thrust_sign_check.py                 # 4 基すべて (既定 duty 0.15)
    python3 thrust_sign_check.py --ch lf rf      # 基を絞る
    python3 thrust_sign_check.py --duty 0.2      # 反応が小さいとき
    python3 thrust_sign_check.py --dry           # 指令を出さず手順と予測だけ表示

**機体は水に浮かべ、回れる程度に緩く係留すること。** 固定すると角速度が出ず判定できない。
姿勢制御ノード (classical_attitude / rl_attitude_node) は**止めてから**動かすこと —
同じ `/cmd/direct` を取り合う。

判定のしかた:
  各基について「+duty を T 秒」「−duty を T 秒」を打ち、その間の**角速度の変化量 Δω**
  を測る。Δω は τ を時間積分したものなので、微分するより雑音に強い。
  `Δω(+) − Δω(−)` を取ると浮力ドリフトや初期回転が消え、純粋に推力由来だけが残る。
  これをバンドルの予測モーメント τ と内積して、**正なら正常・負なら反転**。

  水平成分と垂直成分は別々に見る (サーボ角 0° と 60°)。前者が反転していれば推力の向き、
  後者だけが反転していればサーボの正方向 — 切り分けられる。
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from sensor_msgs.msg import Imu
from sinsei_umiusi_msgs.msg import ThrusterOutput, ThrusterRunnable

POSITIONS = ("lf", "lb", "rb", "rf")
CMD_PREFIX = "/cmd/direct/thruster_controller/output_"
HZ = 50.0
Y_UP = np.array([0.0, 1.0, 0.0])          # CAD の上方向


def cad_to_rep103(v):
    """sim/CAD (+X 前, +Y 上, +Z 右舷) -> REP-103 (x 前, y 左, z 上)。"""
    v = np.asarray(v, dtype=float)
    return np.array([v[0], -v[2], v[1]])


def load_geometry(path):
    """バンドルから thrust_axes / pivots_from_com を読む。**制御則は要らない** —
    予測モーメントの向きだけが欲しいので numpy で足りる (umiusi_perception 不要)。"""
    if not path:
        path = str(Path(get_package_share_directory("umiusi_autonomy"))
                   / "config" / "classical_bundle.json")
    c = json.loads(Path(path).read_text())["contract"]
    return path, np.asarray(c["thrust_axes"], float), np.asarray(c["pivots_from_com"], float)


def predicted_torque(axes, pivots, idx, angle_deg):
    """基 idx を duty>0・サーボ angle_deg で回したときのモーメント (REP-103, 単位推力)。"""
    phi = np.radians(angle_deg)
    f = np.cos(phi) * axes[idx] + np.sin(phi) * Y_UP
    return cad_to_rep103(np.cross(pivots[idx], f))


def exhaust_words(axes, idx):
    """servo 0° / duty>0 のとき、**噴流がどちらへ出るか**を日本語にする。

    推力は `thrust_axes[idx]` の向き、**噴流はその反対**。地上ではこれを目で見る
    (推力は見えないが噴流は見える)。CAD は +X 前 / +Z 右舷。
    """
    v = -np.asarray(axes[idx], dtype=float)        # 噴流 = 推力の逆
    fb = "前" if v[0] > 0.05 else ("後ろ" if v[0] < -0.05 else "")
    ps = "右舷" if v[2] > 0.05 else ("左舷" if v[2] < -0.05 else "")
    return (fb + ps) or "真横"


def mount_words(pivots, idx):
    """取り付け位置を日本語に (前後 × 左右)。"""
    r = np.asarray(pivots[idx], dtype=float)
    return ("前" if r[0] > 0 else "後ろ") + ("右舷" if r[2] > 0 else "左舷")


def in_words(tau):
    """予測モーメントを日本語の向きにする。**目視で確かめるための逃げ道** —
    こちらの座標系の取り違えは、この文と実際の動きを見比べれば operator が捕まえられる。
    REP-103: +roll = 右舷が沈む / +pitch = 機首が下がる / +yaw = 上から見て左回り。"""
    names = [("右舷が下がる", "左舷が下がる"), ("機首が下がる", "機首が上がる"),
             ("上から見て左回り", "上から見て右回り")]
    i = int(np.argmax(np.abs(tau)))
    return names[i][0 if tau[i] > 0 else 1]


class Rig(Node):
    def __init__(self):
        super().__init__("thrust_sign_check")
        self.pubs = {p: self.create_publisher(ThrusterOutput, CMD_PREFIX + p, 10)
                     for p in POSITIONS}
        self.gyro = None
        self.n_imu = 0
        self.create_subscription(Imu, "/state/imu", self._on_imu, 10)

    def _on_imu(self, m):
        g = m.angular_velocity
        self.gyro = np.array([g.x, g.y, g.z])
        self.n_imu += 1

    def send(self, pos=None, duty=0.0, angle=0.0):
        """pos の 1 基だけ駆動し、他はゼロ出力 (runnable は保つ)。"""
        for p in POSITIONS:
            out = ThrusterOutput()
            out.runnable = ThrusterRunnable(esc=True, servo=True)
            on = (p == pos)
            out.duty_cycle = float(duty if on else 0.0)
            out.angle = float(angle if on else 0.0)
            self.pubs[p].publish(out)

    def detach(self):
        for p in POSITIONS:
            out = ThrusterOutput()
            out.runnable = ThrusterRunnable(esc=False, servo=False)
            self.pubs[p].publish(out)

    def spin_for(self, seconds, **kw):
        """指令を 50 Hz で出しつづけながら、その間の gyro を集める。"""
        samples = []
        t_end = time.time() + seconds
        while time.time() < t_end:
            self.send(**kw)
            rclpy.spin_once(self, timeout_sec=1.0 / HZ)
            if self.gyro is not None:
                samples.append(self.gyro.copy())
        return np.array(samples) if samples else np.zeros((0, 3))


def delta_omega(samples, edge_frac=0.2):
    """パルス中の角速度の変化量。前後 edge_frac を平均して差を取る (立ち上がりを避ける)。"""
    n = len(samples)
    if n < 10:
        return np.zeros(3)
    k = max(3, int(n * edge_frac))
    return samples[-k:].mean(axis=0) - samples[:k].mean(axis=0)


def ground_check(args, axes, pivots):
    """**地上で**、噴流の向きを目で見て符号を決める。IMU も水も使わない。

    水に浮かべる版 (この下の main) との違い:
      * 決まるのは**各基の推力の符号**だけ。これはそのまま control の `is_forward` になる。
      * **バンドルの幾何 (`thrust_axes`) が正しいかは決まらない。** 幾何が鏡像なら、
        符号を合わせても水中でまた逆になる。そのときは水で測り直すこと。
      * 機体を押さえる必要がない。大きい機体ではこちらが現実的。

    幾何が地上で使える理由: servo 0° なら 4 基とも推力は水平で、**噴流の向きは取り付け
    位置から一意に決まる** (4 基すべて +duty で合力ちょうど 0 の純粋な偶力配置)。
    """
    print("\n" + "=" * 64)
    print("  地上での向き確認 — **水に入れない。** 機体を押さえる必要もない")
    print("=" * 64)
    print("\n**スラスタを空回しする。** 水冷前提の機体なら長く回さないこと —")
    print(f"1 回 {args.pulse:.0f} 秒、duty {args.duty:.2f} に抑えてある。異音・発熱で止める。")
    print("\n噴流は弱いので、**ティッシュ / 紙片 / 手をノズルの後ろにかざして**見る。")
    print("\n各基について、servo 0°・duty>0 で**噴流が出るはずの向き**:")
    print(f"\n  {'基':5}{'取り付け':10}{'噴流の向き':14}")
    for p in args.ch:
        i = POSITIONS.index(p)
        print(f"  {p:5}{mount_words(pivots, i):10}{exhaust_words(axes, i):14}")
    print("\n  (4 基すべて +duty なら合力 0 の純粋な旋回 = **上から見て右回り**。")
    print("   噴流はその反対向きに出る)")
    if args.dry:
        print("\n--dry: ここまで。指令は出していない。")
        return 0
    if not args.yes:
        try:
            input("\n水に入っていないことを確認して Enter (Ctrl-C で中止): ")
        except (EOFError, KeyboardInterrupt):
            print("\n中止")
            return 1

    rclpy.init()
    rig = Rig()
    results = {}
    try:
        for p in args.ch:
            i = POSITIONS.index(p)
            want = exhaust_words(axes, i)
            print(f"\n--- {p} ({mount_words(pivots, i)}) : 噴流は **{want}** へ出るはず")
            input("    Enter で回す: ")
            print(f"    duty +{args.duty:.2f} を {args.pulse:.0f} s ...", end="", flush=True)
            rig.spin_for(args.pulse, pos=p, duty=args.duty, angle=0.0)
            rig.spin_for(0.3, pos=None, duty=0.0, angle=0.0)
            print(" 停止")
            while True:
                a = input(f"    噴流は {want} へ出たか? [y/n/s(skip)]: ").strip().lower()
                if a in ("y", "n", "s"):
                    break
            results[p] = a
    except KeyboardInterrupt:
        print("\n中止された")
    finally:
        for _ in range(10):
            rig.detach()
            rclpy.spin_once(rig, timeout_sec=0.02)
        print("\nゼロ出力 + detach を送信")

    print("\n" + "=" * 64)
    bad = [p for p, a in results.items() if a == "n"]
    skip = [p for p, a in results.items() if a == "s"]
    if skip:
        print(f"  判定していない基: {sorted(skip)} — 残したまま次へ進まないこと")
    if not bad:
        print("  **4 基とも向きは合っている。** `is_forward` は全部 true のままでよい。")
        print("  それでも水中で逆になるなら、**バンドルの幾何** (thrust_axes) を疑う。")
    else:
        print(f"  **向きが逆の基: {sorted(bad)}**")
        print("\n  control の params/controllers.yaml をこうする:")
        for p in POSITIONS:
            v = "false" if p in bad else "true"
            print(f"    thruster_controller_{p}:  is_forward: {v}")
        sign = ", ".join("-1.0" if p in bad else "1.0" for p in POSITIONS)
        print(f"\n  現場で試すだけなら (再起動すると control の値に戻る):")
        print(f"    ros2 param set /classical_attitude thrust_sign '[{sign}]'")
    print("\n  **直したら水に浮かべて確かめること** — 地上で決まるのは符号だけで、")
    print("  幾何 (取り付け角・位置) が合っているかは決まらない:")
    print("    python3 tools/thrust_sign_check.py        # 水中・IMU で 1 基ずつ")
    rclpy.shutdown()
    return 1 if (bad or skip) else 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--ch", nargs="+", choices=POSITIONS, default=list(POSITIONS))
    ap.add_argument("--duty", type=float, default=0.15,
                    help="励起の duty 絶対値。反応が小さければ上げる (上限 0.3)")
    ap.add_argument("--pulse", type=float, default=6.0, help="1 パルスの秒数")
    ap.add_argument("--rest", type=float, default=5.0, help="パルス間の停止秒数 (回転を落ち着かせる)")
    ap.add_argument("--angle", type=float, default=60.0, help="垂直成分を見るときのサーボ角 [deg]")
    ap.add_argument("--bundle", default="", help="classical_bundle.json のパス (既定は同梱)")
    ap.add_argument("--out", default="", help="生データの保存先 JSON")
    ap.add_argument("--dry", action="store_true", help="指令を出さず、手順と予測だけ表示")
    ap.add_argument("--yes", action="store_true", help="確認プロンプトを省略")
    ap.add_argument("--ground", action="store_true",
                    help="**地上で**噴流の向きを目で見て符号を決める (IMU も水も使わない)。"
                         "機体を押さえなくてよいが、決まるのは符号だけで幾何は決まらない")
    args = ap.parse_args()
    if args.ground:
        args.duty = min(abs(args.duty), 0.12)      # 空回しなので更に絞る
        args.pulse = min(args.pulse, 3.0)

    if abs(args.duty) > 0.3:
        print(f"duty {args.duty} は大きすぎる。0.3 以下にすること")
        return 1

    try:
        bundle, axes, pivots = load_geometry(args.bundle)
    except Exception as e:                                   # noqa: BLE001
        print(f"バンドルを読めない ({type(e).__name__}: {e})")
        return 1
    print(f"バンドル: {bundle}")
    if args.ground:
        return ground_check(args, axes, pivots)

    # 予測: 各基・各サーボ角で「どの軸に一番効くか」。判定はこの向きとの内積で行う
    print("\n予測モーメントの向き (REP-103, duty>0 のとき / 単位推力あたり)")
    print("  **右の日本語を目で確かめること。** ここが実際の動きと食い違うなら、"
          "反転より先に座標系の取り違えを疑う。")
    print(f"  {'基':4} {'サーボ':>7}  {'roll':>8}{'pitch':>8}{'yaw':>8}   +duty で起きるはずのこと")
    plan = []
    for p in args.ch:
        i = POSITIONS.index(p)
        for ang in (0.0, args.angle):
            t = predicted_torque(axes, pivots, i, ang)
            print(f"  {p:4} {ang:6.0f}°  {t[0]:+8.3f}{t[1]:+8.3f}{t[2]:+8.3f}   {in_words(t)}")
            plan.append((p, i, ang, t))

    total = len(plan) * 2 * (args.pulse + args.rest)
    print(f"\n手順: {len(args.ch)} 基 × サーボ 2 通り × (+duty / −duty)"
          f" = {len(plan)*2} パルス、約 {total/60:.1f} 分")
    print(f"  duty ±{args.duty}  パルス {args.pulse:.0f} s  停止 {args.rest:.0f} s")
    print("\n**機体を水に浮かべ、回れる程度に緩く係留すること。** 固定すると判定できない。")
    print("**姿勢制御ノードは止めておくこと** (同じ /cmd/direct を取り合う)。")
    if args.dry:
        print("\n--dry: ここまで。指令は出していない。")
        return 0
    if not args.yes:
        try:
            input("\nEnter で開始 (Ctrl-C で中止): ")
        except (EOFError, KeyboardInterrupt):
            print("\n中止")
            return 1

    rclpy.init()
    rig = Rig()
    print("\n/state/imu を待っています...")
    t0 = time.time()
    while rig.gyro is None and time.time() - t0 < 10.0:
        rclpy.spin_once(rig, timeout_sec=0.2)
    if rig.gyro is None:
        print("/state/imu が来ない。control スタックが上がっているか確認すること")
        rclpy.shutdown()
        return 1
    print(f"IMU OK ({rig.n_imu} サンプル)")

    raw, verdicts = [], []
    try:
        for p, i, ang, tau in plan:
            print(f"\n--- {p} サーボ {ang:.0f}°")
            res = {}
            for sgn in (+1, -1):
                rig.spin_for(args.rest, pos=None, duty=0.0, angle=ang)   # 静定
                print(f"    duty {sgn*args.duty:+.2f} を {args.pulse:.0f} s ...", end="", flush=True)
                s = rig.spin_for(args.pulse, pos=p, duty=sgn * args.duty, angle=ang)
                d = delta_omega(s)
                res[sgn] = d
                print(f" Δω = [{d[0]:+.3f} {d[1]:+.3f} {d[2]:+.3f}] rad/s")
            # 差分で浮力ドリフト・初期回転を消す
            diff = res[+1] - res[-1]
            u = tau / (np.linalg.norm(tau) or 1.0)
            score = float(np.dot(diff, u))
            mag = float(np.linalg.norm(diff))
            raw.append({"ch": p, "angle": ang, "tau": tau.tolist(),
                        "dw_pos": res[+1].tolist(), "dw_neg": res[-1].tolist(),
                        "diff": diff.tolist(), "score": score, "mag": mag})
            if mag < 0.02:
                sv = "反応なし"
            elif score > 0:
                sv = "正常"
            else:
                sv = "反転"
            verdicts.append((p, ang, score, mag, sv))
            print(f"    Δω(+)−Δω(−) = [{diff[0]:+.3f} {diff[1]:+.3f} {diff[2]:+.3f}]"
                  f"  予測方向との内積 {score:+.3f}  -> **{sv}**")
    except KeyboardInterrupt:
        print("\n中止された")
    finally:
        for _ in range(10):
            rig.detach()
            rclpy.spin_once(rig, timeout_sec=0.02)
        print("\nゼロ出力 + detach を送信")

    print("\n" + "=" * 62)
    print(f"  {'基':4} {'サーボ':>7} {'内積':>9} {'|Δω|':>8}   判定")
    for p, ang, score, mag, sv in verdicts:
        print(f"  {p:4} {ang:6.0f}° {score:+9.3f} {mag:8.3f}   {sv}")
    bad = {p for p, ang, s, m, sv in verdicts if sv == "反転"}
    weak = {p for p, ang, s, m, sv in verdicts if sv == "反応なし"}
    print()
    if weak:
        print(f"  反応が出なかった: {sorted(weak)} — 係留がきつい / duty が小さい / 回っていない")
    if not bad:
        print("  反転は見つからなかった。**プール bag の結論と矛盾するので、"
              "係留と duty を見直してもう一度取ること。**")
    else:
        print(f"  **反転している基: {sorted(bad)}**")
        h = {p for p, ang, s, m, sv in verdicts if sv == "反転" and ang == 0.0}
        v = {p for p, ang, s, m, sv in verdicts if sv == "反転" and ang != 0.0}
        if h and h == v:
            print("  水平・垂直の両方が反転 -> **推力ベクトルごと反転**"
                  " (ペラの回転方向 / モータ相 / ESC の逆転設定)")
        elif v - h:
            print(f"  垂直だけ反転: {sorted(v - h)} -> **サーボの正方向**"
                  " (`servo_sign` パラメータで直せる)")
        elif h - v:
            print(f"  水平だけ反転: {sorted(h - v)} -> 推力軸の向き (取り付け角 / バンドルの thrust_axes)")

    # --- 配線の入れ替わり検出 ---------------------------------------------------
    # sim の契約 (configs/umiusi.yaml) は rf->id3 / rb->id4、control の
    # params/controllers.yaml は rb->id3 / rf->id4 で**食い違っている**。どちらが実機かは
    # 測らないと分からないので、測った応答が「別の基の予測」に、より合っていないかを見る。
    print("\n-- 配線の入れ替わり検出 (サーボ 60° の応答を 4 基の予測と総当たり)")
    rows = [r for r in raw if r["angle"] != 0.0]
    if len(rows) < 2:
        print("  サーボ 60° の測定が足りないので判定しない")
    else:
        for r in rows:
            diff = np.array(r["diff"])
            if np.linalg.norm(diff) < 0.02:
                print(f"  {r['ch']:4}: 反応が小さく判定できない")
                continue
            scores = {}
            for q in POSITIONS:
                t = predicted_torque(axes, pivots, POSITIONS.index(q), r["angle"])
                u = t / (np.linalg.norm(t) or 1.0)
                scores[q] = abs(float(np.dot(diff / np.linalg.norm(diff), u)))
            best = max(scores, key=scores.get)
            mark = "" if best == r["ch"] else f"   <- **{best} の予測に一番近い。配線の入れ替わりを疑う**"
            print(f"  {r['ch']:4}: " + "  ".join(f"{q}={scores[q]:.2f}" for q in POSITIONS) + mark)

    if args.out:
        Path(args.out).write_text(json.dumps(raw, indent=2))
        print(f"\n生データ: {args.out}")
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
