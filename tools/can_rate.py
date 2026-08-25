#!/usr/bin/env python3
"""CAN フレームの送信間隔を測る。**ビルド不要** (can-utils の candump を呼ぶだけ)。

サーボが振動する件の切り分け用。`sinsei_umiusi_control` の `can_model.cpp` は
**1 制御ループにつき CAN フレームを 1 つしか送らない** (16 ループで
esc_allowed / esc_duty / servo_allowed / servo_angle × 4 基を 1 周) ので、
`controller_manager` が 100 Hz なら **各サーボの角度更新は 6.25 Hz (160 ms)** になる計算。
VESC 側のタイムアウトがこれより短いと「受信 → タイムアウト → 中立」で振動する。

    python3 tools/can_rate.py                    # 10 秒測って ID ごとの間隔を出す
    python3 tools/can_rate.py --seconds 30
    python3 tools/can_rate.py --iface can0 --all # 既知以外の ID も表示する

CAN ID は `(command_id << 8) | vesc_id` (vesc_model.cpp)。
VESC ID は既定で 124..127 = lf / lb / rb / rf (launch_args.yaml)。
"""
from __future__ import annotations

import argparse
import re
import statistics
import subprocess
import sys
from collections import defaultdict

# command_id (vesc_model.hpp の VescSimpleCommandID)
COMMANDS = {0x00: "esc_duty", 0x03: "esc_rpm", 0x45: "servo_angle"}
# vesc_id -> 位置 (launch_args.yaml の vesc1..4_id 既定値)
VESC = {124: "lf", 125: "lb", 126: "rb", 127: "rf"}

# " (1750000000.123456)  can0  457C   [4]  00 00 80 00"
LINE = re.compile(r"\((\d+\.\d+)\)\s+(\S+)\s+([0-9A-Fa-f]+)\s*#?\s*\[?")


def describe(can_id: int) -> tuple[str, str]:
    """CAN ID から (種別, 位置) を返す。未知なら ("?", "?")。"""
    cmd = COMMANDS.get((can_id >> 8) & 0xFF, "?")
    pos = VESC.get(can_id & 0xFF, "?")
    return cmd, pos


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--iface", default="can0")
    ap.add_argument("--seconds", type=float, default=10.0)
    ap.add_argument("--all", action="store_true", help="既知以外の ID も表示する")
    ap.add_argument("--expect-hz", type=float, default=6.25,
                    help="期待する周波数 (既定は 100Hz/16 = 現在の実装の計算値)")
    a = ap.parse_args()

    print(f"{a.iface} を {a.seconds:.0f} 秒観測します (Ctrl-C で中断)…")
    print("  ※ この間 thruster_cmd.py などで指令を出し続けてください\n")
    try:
        proc = subprocess.Popen(
            ["candump", "-ta", a.iface], stdout=subprocess.PIPE, stderr=subprocess.PIPE,
            text=True, bufsize=1)
    except FileNotFoundError:
        print("candump が見つかりません。`sudo apt install can-utils`", file=sys.stderr)
        return 1

    stamps: dict[int, list[float]] = defaultdict(list)
    t0 = None
    try:
        for line in proc.stdout:
            m = LINE.search(line)
            if not m:
                continue
            t = float(m.group(1))
            try:
                can_id = int(m.group(3), 16)
            except ValueError:
                continue
            if t0 is None:
                t0 = t
            stamps[can_id].append(t)
            if t - t0 >= a.seconds:
                break
    except KeyboardInterrupt:
        print("\n中断しました")
    finally:
        proc.terminate()
        try:
            proc.wait(timeout=2)
        except subprocess.TimeoutExpired:
            proc.kill()

    if not stamps:
        print("フレームを 1 つも受信しませんでした。"
              "can0 が UP か、control が起動しているか確認してください")
        return 1

    rows = []
    for can_id, ts in stamps.items():
        cmd, pos = describe(can_id)
        if not a.all and cmd == "?" and pos == "?":
            continue
        if len(ts) < 2:
            rows.append((can_id, cmd, pos, len(ts), None, None, None))
            continue
        d = [(b - a_) * 1000.0 for a_, b in zip(ts, ts[1:])]     # ms
        rows.append((can_id, cmd, pos, len(ts), statistics.mean(d),
                     min(d), max(d)))
    rows.sort(key=lambda r: (r[1], r[2]))

    print(f"{'CAN ID':>8} {'種別':<12} {'位置':<4} {'件数':>6} "
          f"{'平均間隔':>10} {'最小':>8} {'最大':>8} {'周波数':>9}")
    print("-" * 74)
    for can_id, cmd, pos, n, mean, lo, hi in rows:
        if mean is None:
            print(f"  0x{can_id:04X} {cmd:<12} {pos:<4} {n:>6}        (2 件未満)")
            continue
        hz = 1000.0 / mean if mean > 0 else 0.0
        print(f"  0x{can_id:04X} {cmd:<12} {pos:<4} {n:>6} "
              f"{mean:>9.1f}ms {lo:>7.1f} {hi:>7.1f} {hz:>8.2f}Hz")

    servo = [r for r in rows if r[1] == "servo_angle" and r[4]]
    print()
    if not servo:
        print("  servo_angle のフレームが見つかりません。"
              "サーボ指令を出しているか、servo_allowed が true かを確認してください")
        return 1
    hz = statistics.mean(1000.0 / r[4] for r in servo)
    print(f"  servo_angle の平均周波数: {hz:.2f} Hz "
          f"(計算上の期待値 {a.expect_hz:.2f} Hz)")
    if abs(hz - a.expect_hz) < a.expect_hz * 0.3:
        print("  → **計算どおり**。`can_model.cpp` の 1 ループ 1 パケット方式が効いている。")
        print("     VESC 側のタイムアウトがこれより短ければ振動の原因になる")
    else:
        print("  → 計算と違う。スケジューリング以外の原因を当たること")
    return 0


if __name__ == "__main__":
    sys.exit(main())
