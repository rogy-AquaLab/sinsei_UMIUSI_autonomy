#!/usr/bin/env python3
"""指定トピックの周期と CPU/温度を、決まった時間だけ確実に測る。

`ros2 topic hz` を都度パースする方式は、上流の launch が寿命切れで消えていても
それに気づけず、入力より出力が速いといった辻褄の合わない値を出してしまう。
このツールは自分で購読して数え、同時に「publisher が存在するか」も報告するので、
測定が有効かどうかを取り違えない。

    python3 bench_rates.py --duration 20 \
        /state/imu /front_cam/image_raw /perception_node/detections

出力は 1 行 1 トピックの表 + CPU 使用率 + 温度。--json で機械可読。
"""
from __future__ import annotations

import argparse
import json
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from rosidl_runtime_py.utilities import get_message


def _cpu_sample():
    with open("/proc/stat") as f:
        parts = [float(x) for x in f.readline().split()[1:]]
    idle = parts[3] + parts[4]
    return sum(parts), idle


def _temp_c():
    try:
        with open("/sys/class/thermal/thermal_zone0/temp") as f:
            return int(f.read().strip()) / 1000.0
    except Exception:  # noqa: BLE001
        return float("nan")


class Bench(Node):
    def __init__(self, topics, duration, discovery_sec=4.0):
        super().__init__("bench_rates")
        # ディスカバリ待ち。これを省くと「publisher は居るのに購読が張れず 0 Hz」という
        # 誤った結果になる (このツールを作った動機そのもの)
        t_end = time.time() + discovery_sec
        while time.time() < t_end:
            rclpy.spin_once(self, timeout_sec=0.1)
        self.duration = duration
        self.counts = {t: 0 for t in topics}
        self.first = {t: None for t in topics}
        self.last = {t: None for t in topics}
        self.npub = {t: 0 for t in topics}
        self._subs = []
        # BEST_EFFORT で購読すると RELIABLE/BEST_EFFORT どちらの publisher とも繋がる
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=10)
        names = dict(self.get_topic_names_and_types())
        for t in topics:
            if t not in names:
                self.get_logger().warning(f"{t}: トピックが存在しません")
                continue
            try:
                mt = get_message(names[t][0])
            except Exception as e:  # noqa: BLE001
                self.get_logger().warning(f"{t}: 型を解決できません ({e})")
                continue
            self._subs.append(self.create_subscription(mt, t, self._mk(t), qos))
        self.t0 = time.time()

    def _mk(self, topic):
        def cb(_msg):
            now = time.time()
            if self.first[topic] is None:
                self.first[topic] = now
            self.last[topic] = now
            self.counts[topic] += 1
        return cb

    def snapshot_publishers(self):
        for t in self.counts:
            self.npub[t] = self.count_publishers(t)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("topics", nargs="+")
    ap.add_argument("--duration", type=float, default=20.0)
    ap.add_argument("--label", default="")
    ap.add_argument("--json", action="store_true")
    ap.add_argument("--discovery", type=float, default=4.0,
                    help="購読を張る前のディスカバリ待ち [s]")
    a = ap.parse_args()

    rclpy.init()
    node = Bench(a.topics, a.duration, discovery_sec=a.discovery)
    c0 = _cpu_sample()
    end = time.time() + a.duration
    while time.time() < end and rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.1)
    c1 = _cpu_sample()
    node.snapshot_publishers()

    dtot = c1[0] - c0[0]
    idle_pct = 100.0 * (c1[1] - c0[1]) / dtot if dtot > 0 else float("nan")
    rows = []
    for t in a.topics:
        n = node.counts.get(t, 0)
        f, l = node.first.get(t), node.last.get(t)
        rate = (n - 1) / (l - f) if (n > 1 and l and f and l > f) else 0.0
        rows.append({"topic": t, "rate_hz": round(rate, 3), "count": n,
                     "publishers": node.npub.get(t, 0)})

    result = {"label": a.label, "duration_s": a.duration,
              "cpu_idle_pct": round(idle_pct, 1), "cpu_used_pct": round(100 - idle_pct, 1),
              "temp_c": round(_temp_c(), 1), "topics": rows}

    if a.json:
        print(json.dumps(result, ensure_ascii=False))
    else:
        if a.label:
            print(f"=== {a.label} ===")
        print(f"  {'topic':<42} {'Hz':>8} {'件数':>7} {'pub':>4}")
        for r in rows:
            warn = "  ← publisher なし" if r["publishers"] == 0 else ""
            print(f"  {r['topic']:<42} {r['rate_hz']:>8.2f} {r['count']:>7} {r['publishers']:>4}{warn}")
        print(f"  CPU 使用 {result['cpu_used_pct']}%  (アイドル {result['cpu_idle_pct']}%)"
              f"   温度 {result['temp_c']}C   計測 {a.duration:.0f}s")

    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == "__main__":
    main()
