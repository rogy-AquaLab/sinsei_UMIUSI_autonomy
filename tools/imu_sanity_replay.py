#!/usr/bin/env python3
"""記録した bag の /state/imu に IMU サニティフィルタをかけ直して評価する。

ノードの既定は「検出するが破棄しない」(`imu_sanity_enforce: false`) だが、**このツールは
破棄する側で回す** — 「その閾値で弾いたら何をどれだけ失うか」を見るためのもの。

実機で測り直さずに、**手元の PC で閾値を振って**棄却の出かたを比べるためのもの。
`rl_attitude_node` と同じ `ImuSanity` をそのまま通すので、実機の挙動と一致する。

    # 実機側 (記録は既存の record_run.sh を使う)
    ./tools/record_run.sh --bag-only --name imu-motion     # Ctrl-C で停止

    # PC 側
    python3 tools/imu_sanity_replay.py ~/data/imu/20260821-xxxx-imu-motion/bag
    python3 tools/imu_sanity_replay.py <bag> --sweep       # 閾値を振って比べる

`umiusi_rl_control` がインストールされていない PC では、リポジトリの
`umiusi_rl_control/` を PYTHONPATH に足せば動く (imu_sanity.py は ROS 非依存):

    PYTHONPATH=umiusi_rl_control python3 tools/imu_sanity_replay.py <bag>
"""
from __future__ import annotations

import argparse
import math
import sys

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

import rosbag2_py

from umiusi_rl_control.imu_sanity import ImuSanity, angle_between

MOVING_GYRO = 0.15   # これを超えたら「動かしている」とみなす [rad/s]


def pct(xs, p):
    if not xs:
        return 0.0
    s = sorted(xs)
    return s[min(len(s) - 1, int(len(s) * p / 100.0))]


def read_imu(bag: str, topic: str):
    """bag から (stamp_ns, quat_wxyz, gyro_xyz) を順に返す。"""
    reader = rosbag2_py.SequentialReader()
    opened = False
    for sid in ("mcap", "sqlite3", ""):
        try:
            reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id=sid),
                        rosbag2_py.ConverterOptions("", ""))
            opened = True
            break
        except Exception:                      # noqa: BLE001  ストレージ形式が違うだけ
            reader = rosbag2_py.SequentialReader()
    if not opened:
        raise SystemExit(f"bag を開けません: {bag}")

    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if topic not in types:
        raise SystemExit(f"'{topic}' が bag にありません。含まれるのは: {', '.join(sorted(types))}")
    msg_type = get_message(types[topic])
    while reader.has_next():
        name, data, t_ns = reader.read_next()
        if name != topic:
            continue
        m = deserialize_message(data, msg_type)
        q, g = m.orientation, m.angular_velocity
        yield t_ns, (q.w, q.x, q.y, q.z), (g.x, g.y, g.z)


def run(samples, max_gyro: float, max_step_deg: float):
    """フィルタを通し、統計を返す。samples は read_imu の結果を list にしたもの。"""
    # 評価が目的なので **棄却する側** で回す (ノードの既定は enforce=False)。
    # 「この閾値で弾いたらどれだけ失うか」を見るためのツール
    san = ImuSanity(max_gyro=max_gyro, max_step_deg=max_step_deg, enforce=True)
    gyro_all, gyro_mov, step_all, step_mov = [], [], [], []
    reasons: dict[str, int] = {}
    detail: list[str] = []
    consec = consec_max = stale = 0
    t0 = samples[0][0] if samples else 0

    for t_ns, quat, gyro in samples:
        gmax = max(abs(v) for v in gyro)
        step_deg = 0.0
        prev = san.last
        if prev is not None:
            n = math.sqrt(sum(v * v for v in quat))
            if n > 1e-9:
                step_deg = math.degrees(angle_between(prev.quat, tuple(v / n for v in quat)))
        before = san.resyncs
        _, reason = san.update(quat, gyro)
        resynced = san.resyncs > before

        # 統計は **採用されたサンプルだけ** で取る。化けサンプル (フルスケール 35.74 や
        # ノルム 0 の跳躍) を混ぜると max がそれに支配され、「正常な運動が閾値まで
        # どれくらい余裕があるか」が読めなくなる。棄却分は reasons / detail で見る。
        # 再同期したサンプルの step は「飛ぶ前の姿勢との差」なので運動の統計ではない
        if reason is None and not resynced:
            gyro_all.append(gmax)
            step_all.append(step_deg)
            if gmax > MOVING_GYRO:
                gyro_mov.append(gmax)
                step_mov.append(step_deg)

        if reason is not None:
            key = reason.split(" (")[0]
            reasons[key] = reasons.get(key, 0) + 1
            consec += 1
            consec_max = max(consec_max, consec)
            if consec == san.stale_after + 1:
                stale += 1
            if len(detail) < 20:
                detail.append(f"t={(t_ns - t0) / 1e9:6.2f}s  gyro={gmax:6.2f}  "
                              f"step={step_deg:6.1f}deg  {reason}")
        else:
            consec = 0

    return {"san": san, "gyro_all": gyro_all, "gyro_mov": gyro_mov,
            "step_all": step_all, "step_mov": step_mov, "reasons": reasons,
            "detail": detail, "consec_max": consec_max, "stale": stale}


def report(r, max_gyro, max_step_deg):
    san = r["san"]
    n = san.accepted + san.rejected
    nm = len(r["gyro_mov"])
    print("=" * 72)
    print(f"  {n} サンプル / 採用 {san.accepted} / うち動作中 (|gyro| > {MOVING_GYRO}): {nm}")
    print(f"  閾値: max_gyro {max_gyro} rad/s / max_step_deg {max_step_deg} deg")
    print()
    print(f"  棄却 {san.rejected} 件 ({san.reject_ratio:.3%})"
          f" / 連続棄却の最大 {r['consec_max']} 回"
          f" / 再同期 {san.resyncs} 回 (IMU の姿勢基準が飛んだ回数)")
    for k, v in sorted(r["reasons"].items(), key=lambda kv: -kv[1]):
        print(f"    - {k}: {v} 件")
    if r["detail"]:
        print("  棄却されたサンプル (先頭 20 件):")
        for line in r["detail"]:
            print(f"    {line}")
    print()
    print("  以下は採用されたサンプルのみ (棄却分は上の一覧)")
    print(f"  {'':22} {'p50':>8} {'p95':>8} {'p99':>8} {'max':>8}   閾値      余裕")
    for label, xs, thr in (("角速度 [rad/s] 全体", r["gyro_all"], max_gyro),
                           ("角速度 [rad/s] 動作中", r["gyro_mov"], max_gyro),
                           ("姿勢変化 [deg] 全体", r["step_all"], max_step_deg),
                           ("姿勢変化 [deg] 動作中", r["step_mov"], max_step_deg)):
        if not xs:
            continue
        mx = max(xs)
        margin = (thr / mx) if mx > 1e-9 else float("inf")
        print(f"  {label:22} {pct(xs, 50):8.2f} {pct(xs, 95):8.2f} {pct(xs, 99):8.2f}"
              f" {mx:8.2f}   {thr:6.1f}   {margin:5.1f} 倍")
    print("=" * 72)


def sweep(samples, max_gyro, max_step_deg):
    print("\n== 閾値スイープ ==")
    print(f"\n  max_gyro を振る (max_step_deg={max_step_deg} 固定)")
    print(f"  {'max_gyro':>10} {'棄却':>8} {'率':>9} {'連続最大':>10} {'stale':>7}")
    for g in (1.0, 2.0, 3.0, 5.0, 7.0, 10.0, 15.0, 20.0):
        r = run(samples, g, max_step_deg)
        mark = "  <- 現在" if abs(g - max_gyro) < 1e-9 else ""
        print(f"  {g:10.1f} {r['san'].rejected:8d} {r['san'].reject_ratio:8.3%}"
              f" {r['consec_max']:10d} {r['stale']:7d}{mark}")

    print(f"\n  max_step_deg を振る (max_gyro={max_gyro} 固定)")
    print(f"  {'max_step':>10} {'棄却':>8} {'率':>9} {'連続最大':>10} {'stale':>7}")
    for s in (5.0, 10.0, 15.0, 20.0, 30.0, 45.0, 60.0, 90.0):
        r = run(samples, max_gyro, s)
        mark = "  <- 現在" if abs(s - max_step_deg) < 1e-9 else ""
        print(f"  {s:10.1f} {r['san'].rejected:8d} {r['san'].reject_ratio:8.3%}"
              f" {r['consec_max']:10d} {r['stale']:7d}{mark}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag", help="record_run.sh が作った bag ディレクトリ")
    ap.add_argument("--topic", default="/state/imu")
    ap.add_argument("--max-gyro", type=float, default=10.0)
    ap.add_argument("--max-step-deg", type=float, default=30.0)
    ap.add_argument("--sweep", action="store_true", help="閾値を振って棄却の出かたを比べる")
    a = ap.parse_args()

    samples = list(read_imu(a.bag, a.topic))
    if not samples:
        raise SystemExit(f"'{a.topic}' のメッセージが 0 件です")
    dur = (samples[-1][0] - samples[0][0]) / 1e9
    print(f"{a.bag}\n  {len(samples)} サンプル / {dur:.1f} 秒 ({len(samples) / dur:.1f} Hz)\n")

    r = run(samples, a.max_gyro, a.max_step_deg)
    report(r, a.max_gyro, a.max_step_deg)
    if a.sweep:
        sweep(samples, a.max_gyro, a.max_step_deg)

    ng = 0
    dur = (samples[-1][0] - samples[0][0]) / 1e9
    if r["san"].reject_ratio > 0.01:
        print("\n  ⚠ 棄却率が 1% を超えています。閾値の見直しを検討してください"); ng += 1
    if r["consec_max"] > r["san"].stale_after + 1:
        print(f"\n  ⚠ 連続棄却が {r['consec_max']} 回。再同期が効いていません"); ng += 1
    if r["san"].resyncs and dur > 0 and r["san"].resyncs / (dur / 60.0) > 2.0:
        print(f"\n  ⚠ IMU の姿勢基準の飛びが {r['san'].resyncs} 回 / {dur / 60:.1f} 分。"
              "IMU 側 (キャリブレーション) を疑ってください"); ng += 1
    if ng == 0:
        if r["san"].resyncs:
            print(f"\n  問題なし — IMU が {r['san'].resyncs} 回飛んだが、"
                  f"いずれも {r['consec_max']} サンプルで再同期して復帰している")
        else:
            print("\n  問題なし — この記録の範囲では正常サンプルは弾かれていません")
    return 1 if ng else 0


if __name__ == "__main__":
    sys.exit(main())
