#!/usr/bin/env python3
"""bag_check — 実験直後にプールサイドで bag を検品する (帰ってから使えないと知るのを防ぐ)。

較正計画 (Umiusi_sim docs/calibration_plan.md, issue #15 C 表) の bag 要件をその場で確認する:

  * 必須トピックが入っていてレートが出ているか (/state/imu ~50 Hz, /cmd/direct/... など)
  * **前後の静止 5 秒** が取れているか (較正フィットの基準線。gyro RMS で判定)
  * IMU の化けサンプル率 (ゼロクォータニオン / gyro フルスケール張り付き)
  * **衝突らしき gyro スパイク** の検出と時刻表示 — 狭いプールでは壁ヒットが混ざる。
    該当区間は较正 / world model 学習データから除外するので、時刻をメモしておく
  * teleop / world model 用 (--profile teleop): 励起カバレッジ — duty の符号両方・
    振幅ビン・サーボ可動域をどれだけ使ったか。偏った bag はモデルが偏る

使い方 (PC でも Pi でも。ROS 2 環境を source しておく):
    python3 tools/bag_check.py <bag ディレクトリ>                 # 実験 bag の共通チェック
    python3 tools/bag_check.py <bag> --profile teleop            # ランダム teleop / world model 用
    python3 tools/bag_check.py <bag> --profile static            # 静置・傾斜解放 (実験 5) 用
終了コード: 0 = PASS (警告のみ含む) / 1 = FAIL あり。
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

IMU_TOPIC = "/state/imu"
CMD_TOPICS = tuple(f"/cmd/direct/thruster_controller/output_{p}" for p in ("lf", "lb", "rb", "rf"))
STILL_S = 5.0            # 前後に要求する静止時間 [s]
STILL_GYRO_RMS = 0.05    # 静止判定の gyro RMS 上限 [rad/s]
BUMP_JUMP = 2.0          # 衝突らしさ: 連続サンプル間の |gyro| ジャンプ [rad/s]
GLITCH_GYRO = 9.9        # フルスケール張り付き検出 [rad/s]


def read_bag(bag_dir):
    """bag を読んで {topic: (t[s] array, msgs list)} を返す (必要トピックだけ)。"""
    from rclpy.serialization import deserialize_message
    from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
    from rosidl_runtime_py.utilities import get_message

    reader = SequentialReader()
    reader.open(StorageOptions(uri=str(bag_dir), storage_id=""),
                ConverterOptions(input_serialization_format="cdr",
                                 output_serialization_format="cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    keep = {IMU_TOPIC, *CMD_TOPICS}
    out = {t: ([], []) for t in keep if t in types}
    msg_cls = {t: get_message(types[t]) for t in out}
    while reader.has_next():
        topic, data, stamp_ns = reader.read_next()
        if topic in out:
            out[topic][0].append(stamp_ns * 1e-9)
            out[topic][1].append(deserialize_message(data, msg_cls[topic]))
    return {t: (np.asarray(ts), msgs) for t, (ts, msgs) in out.items()}, types


class Report:
    def __init__(self):
        self.fails = 0

    def line(self, ok, label, detail, warn=False):
        mark = "OK  " if ok else ("WARN" if warn else "FAIL")
        if not ok and not warn:
            self.fails += 1
        print(f"[{mark}] {label}: {detail}")


def main():
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("bag", help="rosbag2 ディレクトリ")
    ap.add_argument("--profile", choices=("run", "teleop", "static"), default="run",
                    help="run=通常実験 / teleop=励起カバレッジも見る / static=静置解放 (指令なしで良い)")
    ap.add_argument("--still", type=float, default=STILL_S, help="要求する前後静止時間 [s]")
    args = ap.parse_args()

    bag = Path(args.bag)
    if not bag.exists():
        sys.exit(f"bag がありません: {bag}")
    data, types = read_bag(bag)
    rep = Report()

    # --- IMU ---
    if IMU_TOPIC not in data or len(data[IMU_TOPIC][0]) == 0:
        rep.line(False, "imu", f"{IMU_TOPIC} が bag にありません")
        sys.exit(1)
    ts, msgs = data[IMU_TOPIC]
    dur = ts[-1] - ts[0]
    rate = len(ts) / max(dur, 1e-9)
    rep.line(dur >= 2 * args.still + 3.0, "duration",
             f"{dur:.1f} s (前後静止 {args.still:.0f} s + 本体が入る長さか)")
    rep.line(rate >= 40.0, "imu rate", f"{rate:.1f} Hz ({len(ts)} msgs)")
    gap = float(np.max(np.diff(ts))) if len(ts) > 1 else 0.0
    rep.line(gap < 0.5, "imu gaps", f"最大欠落 {gap * 1e3:.0f} ms", warn=gap >= 0.2)

    gyro = np.array([[m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z]
                     for m in msgs])
    quat = np.array([[m.orientation.w, m.orientation.x, m.orientation.y, m.orientation.z]
                     for m in msgs])
    gn = np.linalg.norm(gyro, axis=1)

    # 化けサンプル: ゼロ quat / gyro フルスケール
    qn = np.linalg.norm(quat, axis=1)
    glitch = int(np.sum(qn < 1e-6) + np.sum(np.max(np.abs(gyro), axis=1) > GLITCH_GYRO))
    frac = glitch / len(ts)
    rep.line(frac < 0.02, "imu glitches",
             f"{glitch} / {len(ts)} ({frac:.2%}) — ゼロ quat + gyro 張り付き", warn=frac < 0.05)

    # 前後の静止 (較正の基準線)
    for name, sel in (("head still", ts < ts[0] + args.still),
                      ("tail still", ts > ts[-1] - args.still)):
        rms = float(np.sqrt(np.mean(gn[sel] ** 2))) if np.any(sel) else float("nan")
        rep.line(rms < STILL_GYRO_RMS, name,
                 f"gyro RMS {rms:.3f} rad/s (< {STILL_GYRO_RMS}) — 較正の基準線")

    # 衝突らしき gyro ジャンプ (壁ヒット)。連続する化けでないサンプル間の跳び
    ok_samp = (qn > 1e-6) & (np.max(np.abs(gyro), axis=1) <= GLITCH_GYRO)
    ji = np.where(ok_samp[1:] & ok_samp[:-1] & (np.abs(np.diff(gn)) > BUMP_JUMP))[0]
    bump_t = []
    for i in ji:
        t = ts[i + 1] - ts[0]
        if not bump_t or t - bump_t[-1] > 1.0:   # 1 秒以内の連続は同じ衝突
            bump_t.append(t)
    rep.line(len(bump_t) == 0, "bumps",
             ("なし" if not bump_t else
              f"{len(bump_t)} 回 @ " + ", ".join(f"{t:.1f}s" for t in bump_t[:10])
              + " — 較正/world model からこの区間を除外"),
             warn=True)

    # --- 指令 (static プロファイルでは任意) ---
    have_cmd = [t for t in CMD_TOPICS if t in data and len(data[t][0]) > 0]
    if args.profile == "static":
        rep.line(True, "cmd", f"{len(have_cmd)}/4 ch (static では未使用で OK)")
    else:
        rep.line(len(have_cmd) == 4, "cmd topics", f"{len(have_cmd)}/4 ch 記録あり")
        for t in have_cmd:
            r = len(data[t][0]) / max(data[t][0][-1] - data[t][0][0], 1e-9)
            if r < 40.0:
                rep.line(False, "cmd rate", f"{t.rsplit('_', 1)[-1]}: {r:.1f} Hz", warn=True)

    # --- 励起カバレッジ (teleop / world model 用) ---
    if args.profile == "teleop" and have_cmd:
        # ch ごとにメッセージ数が違い得るので flat に連結して見る
        duties = np.concatenate([[m.duty_cycle for m in data[t][1]] for t in have_cmd])
        servos = np.concatenate([[m.angle for m in data[t][1]] for t in have_cmd])
        bins = [(0.05, 0.2), (0.2, 0.4), (0.4, 1.01)]
        for lo, hi in bins:
            frac = float(np.mean((np.abs(duties) >= lo) & (np.abs(duties) < hi)))
            rep.line(frac > 0.05, f"duty {lo:.2f}-{hi:.1f}",
                     f"滞在 {frac:.1%} (各振幅帯を最低 5% は使う)", warn=True)
        both = float(np.mean(np.sign(duties[np.abs(duties) > 0.05])))
        rep.line(abs(both) < 0.7, "duty signs",
                 f"符号バランス {both:+.2f} (±1 に寄り過ぎ = 片方向しか励起していない)", warn=True)
        smin, smax = float(np.min(servos)), float(np.max(servos))
        rep.line(smax - smin > 60.0, "servo range",
                 f"{smin:.0f}..{smax:.0f} deg (可動域を 60 deg 以上使う)", warn=True)

    print("PASS" if rep.fails == 0 else f"FAIL ({rep.fails})")
    sys.exit(0 if rep.fails == 0 else 1)


if __name__ == "__main__":
    main()
