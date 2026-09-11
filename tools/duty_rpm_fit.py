#!/usr/bin/env python3
"""bag から duty 指令 -> rpm 実測の関係を出す。`thrust_curve_exp` の同定に使う。

推力 ∝ rpm^2 は物理として堅いので、`duty -> rpm` の形が分かれば指数が決まる:

  * duty -> rpm が線形        -> 推力 ∝ duty^2  -> thrust_curve_exp ≈ 2.0
  * duty -> rpm が飽和している -> 指数は 2.0 未満

秤も新規実験も要らない。前提と注意は docs/thrust_calibration.md。

使い方:

    python3 tools/duty_rpm_fit.py <bag-dir> [--skip-sec 3] [--min-duty 0.01]

`/state/thruster_state_all` の `duty_cycle` は**指令のエコー**、`rpm` は**実測**。
`esc_mode` が Runnable でないサンプルは指令が送られていないので除外する。
"""

from __future__ import annotations

import argparse
import math

from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from sinsei_umiusi_msgs.msg import ThrusterStateAll

POSITIONS = ("lf", "lb", "rb", "rf")
TOPIC = "/state/thruster_state_all"
RUNNABLE = 1  # util::ThrusterMode::Runnable


def read(uri: str):
    reader = SequentialReader()
    reader.open(StorageOptions(uri=uri, storage_id="mcap"),
                ConverterOptions("cdr", "cdr"))
    out = []
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        if topic != TOPIC:
            continue
        msg = deserialize_message(data, ThrusterStateAll)
        out.append((stamp * 1e-9,
                    [(getattr(msg, p).duty_cycle, getattr(msg, p).rpm,
                      getattr(msg, p).mode.esc) for p in POSITIONS]))
    return out


def fit(xs, ys):
    """原点を通る直線 y = a*x の最小二乗と、決定係数。"""
    sxx = sum(x * x for x in xs)
    if sxx <= 0.0:
        return 0.0, 0.0
    a = sum(x * y for x, y in zip(xs, ys)) / sxx
    mean = sum(ys) / len(ys)
    ss_tot = sum((y - mean) ** 2 for y in ys)
    ss_res = sum((y - a * x) ** 2 for x, y in zip(xs, ys))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0.0 else 1.0
    return a, r2


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--skip-sec", type=float, default=3.0,
                    help="立ち上がりを除外する秒数 (固着の影響が出る)")
    ap.add_argument("--min-duty", type=float, default=0.01,
                    help="これ以下の |duty| は除外 (デッドゾーンは別に見る)")
    args = ap.parse_args()

    rows = read(args.bag)
    if not rows:
        print(f"{TOPIC} が bag に無い: {args.bag}")
        return 1
    t0 = rows[0][0]
    rows = [r for r in rows if r[0] - t0 >= args.skip_sec]
    print(f"samples: {len(rows)}  (先頭 {args.skip_sec:.0f} s を除外)")

    any_data = False
    for i, name in enumerate(POSITIONS):
        armed = [(d, r) for _, per in rows for d, r, m in [per[i]] if m == RUNNABLE]
        if not armed:
            print(f"  {name}: esc_mode が Runnable のサンプルが無い "
                  f"(disarm 状態の bag。回帰には使えない)")
            continue
        live = [(d, r) for d, r in armed if abs(d) >= args.min_duty]
        if len(live) < 20:
            print(f"  {name}: |duty| >= {args.min_duty} のサンプルが {len(live)} 件しかない")
            continue
        any_data = True
        xs = [abs(d) for d, _ in live]
        ys = [abs(r) for _, r in live]
        a, r2 = fit(xs, ys)
        # デッドゾーン: 指令が出ているのに rpm がほぼ 0 の領域
        dead = [d for d, r in armed if abs(d) >= args.min_duty and abs(r) < 1.0]
        print(f"  {name}: n={len(live)} duty [{min(xs):.3f}, {max(xs):.3f}] "
              f"rpm [{min(ys):.0f}, {max(ys):.0f}]")
        print(f"        rpm = {a:.1f} * duty   R^2 = {r2:.4f}"
              f"   {'線形に近い -> exp≈2.0' if r2 > 0.98 else '線形から外れる -> exp<2.0 を疑う'}")
        if dead:
            print(f"        デッドゾーン候補: 指令ありで rpm<1 が {len(dead)} 件 "
                  f"(|duty| 最大 {max(abs(d) for d in dead):.3f})")

    if not any_data:
        print("\n回帰できるデータが無い。arm して走らせた run の bag が要る "
              "(docs/thrust_calibration.md「いま bag に何が入っているか」)。")
        return 1
    print("\n注意: R^2 が高くても低 duty 側だけで測っていれば飽和は見えない。"
          "duty の range を確認すること。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
