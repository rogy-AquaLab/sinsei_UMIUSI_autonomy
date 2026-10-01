#!/usr/bin/env python3
"""「指令していないのに推力が出ている」を bag で測る。

`/cmd/target` の velocity がちょうど 0 の区間だけを取り出し、そのとき
`/cmd/direct/thruster_controller/output_*` が何を出しているかを見る。
出ている推力を **鉛直成分と水平成分に分解**するのが要点:

* 一定の**浮力トリム**は 4 基に同じ向きの鉛直成分として乗る → **和に残る**
* roll/pitch の姿勢フィードバックは基どうしで逆向き（差動）→ **和では大きく打ち消す**

したがって「指令 0 なのに鉛直の和が一方向に偏っている」なら、それは姿勢制御の揺らぎではなく
**定常トリム**。

    python3 tools/idle_thrust_check.py data/20260913-.../bag

注意: `/cmd/direct` に publisher が居ると control の logic は丸ごと迂回される（B-12）。
この経路で録った bag は autonomy の配分器の出力を見ていることになる。
"""
from __future__ import annotations

import math
import statistics as st
import sys

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py

PREFIX = "/cmd/direct/thruster_controller/output_"
STATE_TOPIC = "/state/thruster_state_all"
POSITIONS = ("lf", "lb", "rb", "rf")
RUNNABLE = 1  # util::ThrusterMode::Runnable


def servo_deg(state) -> float:
    """main 系 msgs は angle [deg]、dev-0921 系は commanded_angle [rad]。"""
    if hasattr(state, "commanded_angle"):
        return math.degrees(state.commanded_angle)
    return state.angle


def main(path: str) -> int:
    r = rosbag2_py.SequentialReader()
    r.open(
        rosbag2_py.StorageOptions(uri=path, storage_id=""),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    if "/cmd/target" not in types:
        print("この bag に /cmd/target が無い:", sorted(types))
        return 1
    # direct 経路 (autonomy) は /cmd/direct、control 経路は指令のエコー /state/thruster_state_all
    use_direct = any(t.startswith(PREFIX) for t in types)
    if not use_direct and STATE_TOPIC not in types:
        print(f"{PREFIX}* も {STATE_TOPIC} も無い:", sorted(types))
        return 1
    print(f"出力の出どころ: {PREFIX + '*' if use_direct else STATE_TOPIC}")
    msgs = {k: get_message(v) for k, v in types.items()}

    tgt = None
    last: dict[str, tuple[float, float, bool]] = {}
    vert: list[float] = []
    horiz: list[float] = []
    mag: list[float] = []
    duty_abs: list[float] = []
    phis: list[float] = []

    def accumulate() -> None:
        if tgt is None or tgt > 1e-9 or len(last) < 4:
            return
        live = [(d, a) for d, a, e in last.values() if e and abs(d) > 1e-12]
        if not live:
            return
        vert.append(sum(d * math.sin(math.radians(a)) for d, a in live))
        horiz.append(sum(d * math.cos(math.radians(a)) for d, a in live))
        mag.append(sum(abs(d) for d, a in live))
        duty_abs.extend(abs(d) for d, a in live)
        phis.extend(abs(a) for d, a in live)

    while r.has_next():
        tn, data, _ts = r.read_next()
        if tn == "/cmd/target":
            v = deserialize_message(data, msgs[tn]).velocity
            tgt = abs(v.x) + abs(v.y) + abs(v.z)
        elif use_direct and tn.startswith(PREFIX):
            m = deserialize_message(data, msgs[tn])
            last[tn[len(PREFIX):]] = (m.duty_cycle, m.angle, m.runnable.esc)
            accumulate()
        elif not use_direct and tn == STATE_TOPIC:
            m = deserialize_message(data, msgs[tn])
            for p in POSITIONS:
                th = getattr(m, p)
                last[p] = (th.duty_cycle, servo_deg(th), th.mode.esc == RUNNABLE)
            accumulate()

    n = len(vert)
    print(f"指令 velocity==0 かつ 推力を出しているサンプル: {n}")
    if n == 0:
        print("  該当なし（この bag では指令 0 の区間で推力が出ていない）")
        return 0
    print(f"  net 鉛直 Σ duty*sin(angle) : mean={st.mean(vert):+.4f} med={st.median(vert):+.4f}")
    print(f"  net 水平 Σ duty*cos(angle) : mean={st.mean(horiz):+.4f} med={st.median(horiz):+.4f}")
    print(f"  総推力 Σ|duty|             : mean={st.mean(mag):.4f} med={st.median(mag):.4f} "
          f"max={max(mag):.4f}")
    print(f"  1 基あたり |duty|          : med={st.median(duty_abs):.4f} max={max(duty_abs):.4f}")
    phis.sort()
    over80 = sum(1 for x in phis if x > 80) / len(phis) * 100
    print(f"  |サーボ角|                 : med={phis[len(phis) // 2]:.1f}deg  >80deg {over80:.1f}%")
    print()
    print("  鉛直の和が水平の和より明らかに大きく、符号が一定なら = 定常トリム（姿勢制御ではない）")
    return 0


if __name__ == "__main__":
    sys.exit(main(*sys.argv[1:]))
