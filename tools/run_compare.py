#!/usr/bin/env python3
"""1 本の bag に入った「古典 vs RL」を arm 区間で切り分けて並べる (2026-09-12 の run 用)。

現場は 2 本を**同じ bag** に録ることがある (docs/field_card.md)。どちらが走っていたかは
**トピックからは分からない** — 両方とも同じ `/cmd/direct/...` に出し、`~/arm` は**サービス**
なので topic に残らない。唯一の手掛かりが `/rosout` で、`umiusi_common.arm.ArmState` が
arm/disarm のたびに `ARMED` / `DISARMED` を、logger 名 (= ノード名) 付きで出している。
そこを区間の境界に使う。

    python3 tools/run_compare.py <bag-dir>
    python3 tools/run_compare.py <bag-dir> --settle-sec 5    # 区間頭の過渡を捨てる秒数
    python3 tools/run_compare.py <bag-dir> --segments        # 区間の一覧だけ出して終わる

出すもの (区間ごと):

  * **姿勢**   roll/pitch は水平 (目標クォータニオンが単位元) からの誤差なので絶対値で見る。
               yaw は基準が任意なので**区間頭からの差**で見る。RMS と 95 パーセンタイル。
  * **発振**   誤差の主要周波数 (FFT) と、微分の符号反転レート。手で傾けた復帰と、
               制御が行き過ぎて戻るのを繰り返す発振は**前者が低周波・単発**で分かれる。
  * **飽和**   duty が上限に張り付いた割合と、サーボが ±88 deg に張り付いた割合。
               cap 0.25 では古典も飽和するのが分かっているので (docs/thrust_calibration.md)、
               **どちらが良いかの前に「そもそも比例制御になっていたか」**をこれで見る。

**この道具は判定をしない。** 数字を並べるだけで、良し悪しは現場の所見と突き合わせて決める。
"""

from __future__ import annotations

import argparse
import math
import re
import sys
from collections import defaultdict

import numpy as np
from rcl_interfaces.msg import Log
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from sensor_msgs.msg import Imu
from sinsei_umiusi_msgs.msg import ThrusterOutput

POSITIONS = ("lf", "lb", "rb", "rf")
CMD_PREFIX = "/cmd/direct/thruster_controller/output_"
IMU_TOPIC = "/state/imu"
ROSOUT_TOPIC = "/rosout"
# 比較する 2 つのノード。**ノード名は launch の `name=` で決まる** — 古典は
# classical_attitude、RL だけ `_node` が付く (handover の誤記の出どころ)。
CONTROLLERS = {"classical_attitude": "古典", "rl_attitude_node": "RL"}
# サーボの機械端。これ以上は出せないので、ここに張り付いたら配分が効いていない
SERVO_LIMIT_DEG = 88.0
TYPES = {ROSOUT_TOPIC: Log, IMU_TOPIC: Imu,
         **{CMD_PREFIX + p: ThrusterOutput for p in POSITIONS}}


def read(uri: str):
    """必要なトピックだけを 1 パスで読む。{topic: [(t, msg), ...]}"""
    reader = SequentialReader()
    reader.open(StorageOptions(uri=uri, storage_id="mcap"),
                ConverterOptions("cdr", "cdr"))
    out = defaultdict(list)
    while reader.has_next():
        topic, data, stamp = reader.read_next()
        msg_type = TYPES.get(topic)
        if msg_type is None:
            continue
        out[topic].append((stamp * 1e-9, deserialize_message(data, msg_type)))
    return out


def segments(rosout, t_end):
    """/rosout の ARMED / DISARMED から、ノードごとの arm 区間を作る。

    落ちどころ: **最後の DISARMED が無いことがある** (Ctrl-C で落とすと出ないまま終わる。
    手順書はこれを止め方として挙げているので、当日はむしろこちらが普通)。そのときは
    **bag 全体の終端** `t_end` で閉じる — /rosout の最終行で閉じると、その後も IMU と指令が
    録れているのに区間が 0 s に潰れる。逆に ARMED を挟まない DISARMED は無視する。
    """
    open_at, spans = {}, []
    for t, m in rosout:
        node = m.name.split(".")[0]         # logger 名は "node" か "node.sub"
        if node not in CONTROLLERS:
            continue
        if m.msg.startswith("ARMED"):
            open_at.setdefault(node, t)
        elif m.msg.startswith("DISARMED") and node in open_at:
            spans.append((node, open_at.pop(node), t))
    for node, t in open_at.items():
        spans.append((node, t, t_end))      # 閉じずに終わった区間
    return sorted(spans, key=lambda s: s[1])


# 起動ログの cap。**`@max_duty=` を除く** — バンドル行の
# 「到達速度 @max_duty=0.208 m/s」は cap ではなく「その cap での到達速度」で、
# 素朴に `max_duty=` を拾うとこちらに当たって飽和率が過大に出る (2026-09-12 の bag で踏んだ)。
MAX_DUTY_RE = re.compile(r"(?<![@\w])max_duty=([0-9.]+)")


def max_duty_of(rosout, node, before):
    """`before` 以前で最後に出た起動ログの cap。無ければ None (実測の最大で代用)。

    ノードは 1 本の bag のなかで何度も起動し直される (当日は RL が 4 回)。**区間ごとに
    直前の起動ログを見る** — 最初の 1 件で決め打つと、途中で cap を変えた run を取り違える。
    """
    cap = None
    for t, m in rosout:
        if t > before:
            break
        if m.name.split(".")[0] == node:
            hit = MAX_DUTY_RE.search(m.msg)
            if hit:
                cap = float(hit.group(1))
    return cap


def rpy_deg(msg):
    """sensor_msgs/Imu の orientation -> roll/pitch/yaw [deg] (ZYX)。"""
    q = msg.orientation
    w, x, y, z = q.w, q.x, q.y, q.z
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    s = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(s)
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return np.degrees([roll, pitch, yaw])


# 誤差の標準偏差がこれ未満の軸は「動いていない」として発振を報告しない [deg]。
# 平坦な軸に FFT をかけると量子化ノイズの山が拾われ、**静かな軸ほど高周波に見える**。
# 合成 bag で確認: pitch 一定の区間が 3.9 Hz / 16.7 flips/s と出た。
QUIET_DEG = 0.05
# 主要周波数を採用する条件: ピークがスペクトルの中央値の何倍あるか。
# 白色ノイズなら山はこの比に達しない
PEAK_PROMINENCE = 4.0


def oscillation(t, err):
    """発振の指標: 誤差の主要周波数 [Hz] と、微分の符号反転レート [1/s]。

    手で傾けて戻るのは低周波 (< 0.3 Hz) の単発、制御の発振は上に出る。
    サンプル間隔は 50 Hz 前提だが、欠落があるので実測の中央値で割る。

    **ノイズを発振と読ませない**ために 2 つの歯止めを置く: 振れていない軸は報告しない
    (QUIET_DEG)、山が立っていないスペクトルは周波数を出さない (PEAK_PROMINENCE)。
    符号反転も生の差分だと IMU ノイズで飽和するので、5 サンプルの移動平均にしてから数える。
    """
    nan = float("nan")
    if len(err) < 32:
        return nan, nan
    dt = float(np.median(np.diff(t)))
    if not (dt > 0.0):
        return nan, nan
    e = err - np.mean(err)
    if float(np.std(e)) < QUIET_DEG:
        return nan, nan                      # 動いていない軸。発振も何も無い
    spec = np.abs(np.fft.rfft(e * np.hanning(len(e))))
    freqs = np.fft.rfftfreq(len(e), dt)
    live = freqs >= 0.05                     # DC 近傍はドリフトなので除く
    peak = nan
    if live.any():
        band, med = spec[live], float(np.median(spec[live]))
        if med > 0.0 and float(np.max(band)) >= PEAK_PROMINENCE * med:
            peak = float(freqs[live][int(np.argmax(band))])
    k = 5
    sm = np.convolve(e, np.ones(k) / k, mode="valid")
    d = np.diff(sm)
    flips = int(np.sum(np.sign(d[1:]) * np.sign(d[:-1]) < 0))
    return peak, flips / (t[-1] - t[0])


def attitude_stats(imu, t0, t1):
    rows = [(t, rpy_deg(m)) for t, m in imu if t0 <= t <= t1]
    if len(rows) < 2:
        return None
    t = np.array([r[0] for r in rows])
    rpy = np.array([r[1] for r in rows])
    # roll/pitch の目標は水平 (bundle の目標クォータニオンが単位元) なので絶対値がそのまま誤差。
    # yaw は基準が任意 = 区間頭からの差。±180 をまたぐので unwrap してから引く。
    yaw = np.degrees(np.unwrap(np.radians(rpy[:, 2])))
    err = np.column_stack([rpy[:, 0], rpy[:, 1], yaw - yaw[0]])
    out = {"n": len(rows), "hz": (len(rows) - 1) / (t[-1] - t[0])}
    for i, axis in enumerate(("roll", "pitch", "yaw")):
        out[axis] = {
            "rms": float(np.sqrt(np.mean(err[:, i] ** 2))),
            "p95": float(np.percentile(np.abs(err[:, i]), 95)),
            "max": float(np.max(np.abs(err[:, i]))),
        }
        out[axis]["freq"], out[axis]["flips"] = oscillation(t, err[:, i])
    return out


def actuator_stats(cmds, t0, t1, max_duty):
    """duty / サーボ角の張り付き。cmds は {pos: [(t, ThrusterOutput)]}。"""
    out = {}
    for p in POSITIONS:
        rows = [m for t, m in cmds.get(p, []) if t0 <= t <= t1]
        # runnable が落ちている間は detach (ゼロ出力) なので除く — 混ぜると飽和率が薄まる
        live = [m for m in rows if m.runnable.esc]
        if not live:
            out[p] = None
            continue
        duty = np.abs([m.duty_cycle for m in live])
        angle = np.array([m.angle for m in live])
        cap = max_duty if max_duty else float(np.max(duty))
        out[p] = {
            "n": len(live),
            "duty_mean": float(np.mean(duty)),
            "duty_max": float(np.max(duty)),
            # 上限の 99% 以上を「張り付き」とする。cap ちょうどで丸められるので等号は使わない
            "duty_sat": float(np.mean(duty >= cap * 0.99)) if cap > 0 else float("nan"),
            "angle_mean": float(np.mean(angle)),
            "angle_sat": float(np.mean(np.abs(angle) >= SERVO_LIMIT_DEG)),
        }
    return out


def report(node, label, t0, t1, att, act, max_duty, t_bag0):
    cap = f"{max_duty:.2f}" if max_duty else "?"
    print(f"\n=== {label} ({node})  {t0 - t_bag0:7.1f}..{t1 - t_bag0:7.1f} s "
          f"({t1 - t0:.0f} s, max_duty={cap})")
    if att is None:
        print("  姿勢: /state/imu のサンプルが無い。区間が短すぎるか録れていない")
    else:
        print(f"  姿勢 ({att['n']} サンプル, {att['hz']:.1f} Hz)   "
              f"[roll/pitch は水平からの誤差、yaw は区間頭からの差]")
        print("       axis     RMS     p95     max   主要周波数   符号反転")
        for axis in ("roll", "pitch", "yaw"):
            a = att[axis]
            # 振れていない軸 / 山の立たないスペクトルは「—」。数字を出すと発振と誤読される
            freq = "      —  " if math.isnan(a["freq"]) else f"{a['freq']:8.2f} Hz"
            flips = "       — " if math.isnan(a["flips"]) else f"{a['flips']:8.1f} /s"
            print(f"      {axis:>6} {a['rms']:7.2f} {a['p95']:7.2f} {a['max']:7.2f}"
                  f" {freq} {flips}")
    live = {p: v for p, v in act.items() if v}
    if not live:
        print("  指令: runnable な /cmd/direct が無い。publish:=false の区間か、"
              "指令が出ていない")
        return
    print("  指令   duty平均  duty最大  duty張付  角度平均  角度張付")
    for p in POSITIONS:
        v = act.get(p)
        if not v:
            print(f"    {p}: 指令が無い")
            continue
        print(f"    {p}:  {v['duty_mean']:7.3f}  {v['duty_max']:7.3f}  "
              f"{v['duty_sat'] * 100:6.1f}%  {v['angle_mean']:7.1f}  "
              f"{v['angle_sat'] * 100:6.1f}%")
    sat = np.mean([v["duty_sat"] for v in live.values()])
    if sat > 0.5:
        print(f"    ⚠ duty が {sat * 100:.0f}% の時間で上限に張り付いている。"
              f"この区間は比例制御になっていない — ゲインの比較には使えない")


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--settle-sec", type=float, default=3.0,
                    help="arm 直後の過渡を捨てる秒数 (積分器の立ち上がりが混ざる)")
    ap.add_argument("--min-sec", type=float, default=10.0,
                    help="これより短い区間は報告しない (押し間違いの arm)")
    ap.add_argument("--segments", action="store_true", help="区間の一覧だけ出す")
    args = ap.parse_args()

    try:
        data = read(args.bag)
    except RuntimeError as e:
        print(f"bag を開けない: {args.bag}\n  {e}\n"
              "ディレクトリ (metadata.yaml と .mcap が入っているほう) を渡すこと。"
              "metadata.yaml が無いなら tools/record_run.sh --fix で復元できる。")
        return 1
    rosout = data.get(ROSOUT_TOPIC, [])
    if not rosout:
        print(f"/rosout が bag に無い: {args.bag}\n"
              "区間を切り分けられない。record_run.sh で録ったか確認すること "
              "(arm はサービスなので /rosout 以外に痕跡が無い)。")
        return 1

    t_bag0 = min(t for rows in data.values() for t, _ in rows)
    t_bag1 = max(t for rows in data.values() for t, _ in rows)
    spans = segments(rosout, t_bag1)
    if not spans:
        print("ARMED / DISARMED が /rosout に無い。一度も arm していないか、"
              "別のスタック (control 版の RL) で走っている。\n"
              "control 版の arm は /cmd/thruster_runnable_all に出るので、そちらを見ること。")
        return 1

    print(f"bag: {args.bag}")
    print(f"arm 区間 {len(spans)} 本:")
    for node, t0, t1 in spans:
        print(f"  {CONTROLLERS[node]:>4} ({node})  "
              f"{t0 - t_bag0:7.1f} .. {t1 - t_bag0:7.1f} s  ({t1 - t0:5.0f} s)")
    if args.segments:
        return 0

    imu = data.get(IMU_TOPIC, [])
    cmds = {p: data.get(CMD_PREFIX + p, []) for p in POSITIONS}

    shown = 0
    for node, t0, t1 in spans:
        if t1 - t0 < args.min_sec:
            continue
        caps = {node: max_duty_of(rosout, node, t0)}
        s0 = min(t0 + args.settle_sec, t1)
        att = attitude_stats(imu, s0, t1)
        act = actuator_stats(cmds, s0, t1, caps[node])
        report(node, CONTROLLERS[node], s0, t1, att, act, caps[node], t_bag0)
        shown += 1
    if not shown:
        print(f"\n{args.min_sec:.0f} s 以上の区間が無い。--min-sec を下げて見ること。")
        return 1

    missing = [n for n in CONTROLLERS if not any(s[0] == n for s in spans)]
    for n in missing:
        print(f"\n注意: {CONTROLLERS[n]} ({n}) の区間が無い。片側しか走っていないので"
              "比較にはならない。")
    print("\n数値は現場の所見と突き合わせること。**この道具は良し悪しを判定しない。**")
    return 0


if __name__ == "__main__":
    sys.exit(main())
