#!/usr/bin/env python3
"""bag から **esc の符号反転の頻度と、そこからの立ち上がり時間**を測る。

なぜ要るか: アジマス折返しはユニットの推力をゼロ交差させる。**静止からの起動には
約 2.7 s の死に時間がある**ので (known_issues B-18)、反転のたびに同じコストを払うなら
姿勢制御が成立しない。sim 側の評価では、その仮定で姿勢誤差が 8 倍に悪化した。

  * **頻度**はアロケータの実装で変わる。sim の特異点回避を直すたびに測り直すこと
  * **1 回あたりのコスト**は物理なので、機体が変わらなければ変わらない

**対照を必ず見ること。** 同じ読み方で「静止からの起動」も出す。そこが 2.5〜3.2 s に
ならない bag は、読み方が成立していない (rpm テレメトリが遅すぎる等) ので反転の数字も
信用しない。`rf` は rpm が 2.4〜3.7 Hz しか来ないので `lb` / `rb` を主に見る。

    python3 tools/reversal_check.py <bag-dir>
"""
from __future__ import annotations

import argparse
import sys

import numpy as np
from rcl_interfaces.msg import Log
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from sinsei_umiusi_msgs.msg import ThrusterStateAll

POSITIONS = ("lf", "lb", "rb", "rf")
TOPIC = "/state/thruster_state_all"


def read(path):
    r = SequentialReader()
    r.open(StorageOptions(uri=str(path), storage_id="mcap"),
           ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"))
    rows, armed, t0 = [], None, None
    while r.has_next():
        topic, data, t = r.read_next()
        if t0 is None:
            t0 = t
        ts = (t - t0) / 1e9
        if topic == TOPIC:
            m = deserialize_message(data, ThrusterStateAll)
            row = [ts]
            for p in POSITIONS:
                s = getattr(m, p)
                row += [s.duty_cycle, s.rpm]
            rows.append(row)
        elif topic == "/rosout" and armed is None:
            m = deserialize_message(data, Log)
            if m.name == "classical_attitude" and m.msg.strip() == "ARMED":
                armed = ts
    return np.array(rows), armed


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--hold", type=float, default=0.3,
                    help="反転の前後でこれだけ符号を保っていたら本物とみなす [s]")
    ap.add_argument("--min-duty", type=float, default=0.05)
    ap.add_argument("--recover", type=float, default=0.5,
                    help="反転前の |rpm| の何割まで戻ったら立ち上がったとするか")
    a = ap.parse_args()

    st, armed = read(a.bag)
    if not len(st):
        print(f"{TOPIC} が bag に無い", file=sys.stderr)
        return 1
    # **対照は arm の直後を見る。** 反転の解析は過渡を避けて +5 s から始めるが、
    # 静止からの起動はまさにその 5 s の中で起きるので、**切る前の配列で測る**
    cold = st[st[:, 0] > (armed or 0.0)]
    t_arm = (armed or 0.0) + 5.0
    st = st[st[:, 0] > t_arm]
    if len(st) < 200:
        print("arm 後のサンプルが足りない", file=sys.stderr)
        return 1
    t = st[:, 0]
    span = t[-1] - t[0]
    print(f"arm={armed}  使う区間 {span:.0f}s")

    print("\n## 対照: 静止からの起動 (2.5〜3.2 s に出れば読み方が成立している)")
    tc = cold[:, 0]
    for k, p in enumerate(POSITIONS):
        duty, rpm = cold[:, 1 + 2 * k], cold[:, 2 + 2 * k]
        on = np.where(np.abs(duty) > a.min_duty)[0]
        if not len(on):
            print(f"  {p}: 指令なし")
            continue
        w = np.where((tc > tc[on[0]]) & (np.abs(rpm) > 300))[0]
        print(f"  {p}: {tc[w[0]] - tc[on[0]]:.2f}s" if len(w) else f"  {p}: 立ち上がらず")

    print("\n## 符号反転")
    for k, p in enumerate(POSITIONS):
        duty, rpm = st[:, 1 + 2 * k], st[:, 2 + 2 * k]
        if np.max(np.abs(rpm)) < 200:
            print(f"  {p}: 回っていない")
            continue
        sg = np.sign(duty) * (np.abs(duty) > a.min_duty)
        idx = np.where((sg[:-1] != 0) & (sg[1:] != 0) & (sg[:-1] != sg[1:]))[0] + 1
        evs, pre_duty = [], []
        for i in idx:
            pre = (t >= t[i] - a.hold) & (t < t[i])
            post = (t > t[i]) & (t <= t[i] + a.hold)
            if pre.sum() < 3 or post.sum() < 3:
                continue
            if not (np.all(sg[pre] == sg[i - 1]) and np.all(sg[post] == sg[i])):
                continue
            r0 = np.median(np.abs(rpm[pre]))
            if r0 < 300:
                continue
            w = (t > t[i]) & (t <= t[i] + 6.0)
            if w.sum() < 5:
                continue
            good = (np.sign(rpm[w]) == sg[i]) & (np.abs(rpm[w]) >= a.recover * r0)
            if not good.any():
                continue
            evs.append(t[w][np.argmax(good)] - t[i])
            pre_duty.append(np.median(np.abs(duty[pre])))
        n = len(idx)
        line = f"  {p}: {n} 回 ({n / span:.2f} 回/s) / 測れた {len(evs)} 件"
        if evs:
            e, d = np.array(evs), np.array(pre_duty)
            line += (f"  立ち上がり 中央 {np.median(e):.2f}s p90 {np.percentile(e, 90):.2f}s"
                     f"  反転直前の |duty| 中央 {np.median(d):.3f}")
        else:
            # 条件を満たす事象が少ない = ゼロ付近でチャタリングしている
            line += "  **ゼロ付近でチャタリングしている疑い**"
        print(line)
    print("\n**頻度はアロケータが変わると変わる。** 特異点回避を直したら測り直すこと。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
