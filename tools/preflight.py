#!/usr/bin/env python3
"""競技/実験の直前に、**機体が既知の正しい状態か**を 1 コマンドで確かめる。

現場で踏んだ失敗は「設定が入っていなかった」「入れたつもりが効いていなかった」の 2 種類で、
**どちらもエラーを出さない**。出ないものは見えないので、ここで名指しで確認する。

    python3 tools/preflight.py              # スタックを上げた状態で実行
    python3 tools/preflight.py --teleop     # UI のゲームパッドで操縦する日はこちら

終了コード: 0 = 出艇可 / 1 = **止める** / 2 = 警告のみ。

確認するもの (根拠は docs/known_issues.md の各項):

  B-14  バンドルの水平の符号が control の規約と合っているか / `is_forward` が 4 基とも true か
  B-16  死んでいるスラスタを control と autonomy の両方が同じに認識しているか
  B-17  `cmd_target_yaw_mode` を `ros2 param set` で変えられるバイナリか・rate になっているか
  B-18  cap が 3 基運用に足りるか
  B-12  `/cmd/direct` の publisher が 1 つだけか (2 つあると指令を取り合う)

**この道具は機体を動かさない。** 読むだけ。
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.node import Node

POSITIONS = ("lf", "lb", "rb", "rf")
Y_UP = np.array([0.0, 1.0, 0.0])
ATTITUDE = "/classical_attitude"
CMD_TOPIC = "/cmd/direct/thruster_controller/output_lf"
PRESSURE = "/state/pressure"

OK, WARN, FAIL = "OK  ", "警告", "不可"


class Report:
    def __init__(self):
        self.rows = []

    def add(self, level, title, detail):
        self.rows.append((level, title, detail))

    def dump(self):
        print()
        for level, title, detail in self.rows:
            print(f"  [{level}] {title}")
            for line in detail.splitlines():
                print(f"         {line}")
        bad = sum(1 for level, _, _ in self.rows if level == FAIL)
        warn = sum(1 for level, _, _ in self.rows if level == WARN)
        print()
        if bad:
            print(f"** {bad} 件が不可。出艇しないこと。** (警告 {warn} 件)")
            return 1
        if warn:
            print(f"出艇可。ただし警告 {warn} 件 — 上を読んでから。")
            return 2
        print("すべて OK。出艇可。")
        return 0


def _unpack(v):
    """ParameterValue -> Python の値。**型は全部並べる** — 抜けがあると None になって
    「読めなかった」と区別が付かなくなる (DOUBLE_ARRAY の thrust_sign で踏みかけた)。"""
    return {1: lambda: v.bool_value, 2: lambda: v.integer_value, 3: lambda: v.double_value,
            4: lambda: v.string_value, 5: lambda: list(v.byte_array_value),
            6: lambda: list(v.bool_array_value), 7: lambda: list(v.integer_array_value),
            8: lambda: list(v.double_array_value), 9: lambda: list(v.string_array_value),
            }.get(v.type, lambda: None)()


def get_params(node, ns, names, timeout=3.0):
    """他ノードの param を読む。ノードが居なければ None、個々の未宣言は値が None。

    **1 つずつ問い合わせる。** rclpy の `_get_parameters_callback` は、要求した名前に
    **1 つでも未宣言があると values を空で返す** (`parameter_service.py`)。まとめて聞くと、
    古いバイナリ (= このツールが対象にしている状態そのもの) が「ノードが居ない」に
    化けて、残りの検査が全部飛ぶ。
    """
    cli = node.create_client(GetParameters, f"{ns}/get_parameters")
    try:
        if not cli.wait_for_service(timeout_sec=timeout):
            return None
        out = {}
        for n in names:
            req = GetParameters.Request()
            req.names = [n]
            fut = cli.call_async(req)
            rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
            res = fut.result()
            out[n] = _unpack(res.values[0]) if (res and res.values) else None
        return out
    finally:
        node.destroy_client(cli)


def check_bundle(rep, path):
    """バンドルの水平の符号が control の feed_forward の規約と合っているか。

    規約: **全基の水平出力を正にすると z 軸まわり反時計回り** (control の
    `logic/attitude/feed_forward.hpp` に明記、2026-09-13 の実機実測も反時計回り)。
    水平列は yaw にしかトルクを作らないので、これだけで判定できる。
    """
    try:
        c = json.loads(Path(path).read_text())["contract"]
    except Exception as e:  # noqa: BLE001
        rep.add(FAIL, "バンドルが読めない", f"{path}: {type(e).__name__}: {e}")
        return
    axes = np.asarray(c["thrust_axes"], float)
    piv = np.asarray(c["pivots_from_com"], float)
    yaw = sum(float(np.cross(piv[k], axes[k])[1]) for k in range(4))   # CAD +Y まわり
    if yaw > 0:
        rep.add(OK, "バンドルの水平の符号 (B-14)",
                f"全基 +h で反時計回り (yaw {yaw:+.3f})。control の規約と一致\n{path}")
    else:
        rep.add(FAIL, "バンドルの水平の符号が**旧規約** (B-14)",
                f"全基 +h で時計回り (yaw {yaw:+.3f})。control / 実機と逆。\n"
                f"autonomy を main (7bb1d16 以降) に更新すること。\n{path}")


def check_is_forward(rep, node):
    vals, missing = {}, []
    for p in POSITIONS:
        r = get_params(node, f"/thruster_controller_{p}", ["is_forward", "esc_disabled"])
        if r is None:
            missing.append(p)
        else:
            vals[p] = r
    if missing:
        rep.add(FAIL, "control が応答しない (B-14/B-16)",
                f"{missing} の thruster_controller から param を読めない。control が上がっていない")
        return None
    fwd = {p: vals[p]["is_forward"] for p in POSITIONS}
    dis = {p: vals[p]["esc_disabled"] for p in POSITIONS}
    if all(fwd.values()):
        rep.add(OK, "control の is_forward (B-14)", "4 基とも true")
    else:
        rep.add(FAIL, "control の is_forward が false の基がある (B-14)",
                f"{fwd}\n2026-09-13 の現場で入れた応急処置なら**戻すこと (全 true)**。\n"
                "false のままだと roll / pitch / 深度が反転する (bag で測定済み)")
    dead = [p for p in POSITIONS if dis[p]]
    if len(dead) > 1:
        rep.add(FAIL, "esc_disabled が 2 基以上 (B-16)", f"{dead}。6 自由度の権限が無い")
    elif dead:
        rep.add(WARN, "スラスタ 1 基を殺している (B-16)",
                f"{dead[0]} が esc_disabled。autonomy 側も同じに見えているかは下で確認")
    else:
        rep.add(OK, "esc_disabled (B-16)", "4 基とも生きている扱い")
    return dead


def check_attitude(rep, node, dead, teleop):
    names = ["thrust_sign", "servo_sign", "max_duty", "hold_yaw", "cmd_target_topic",
             "cmd_target_yaw_mode", "live_thrusters", "bundle_path",
             "imu_timeout", "vel_timeout"]
    r = get_params(node, ATTITUDE, names)
    if r is None:
        rep.add(FAIL, "classical_attitude が応答しない",
                f"{ATTITUDE}/get_parameters が返らない。姿勢制御器が上がっていない")
        return None
    core = [n for n in ("live_thrusters", "max_duty", "bundle_path") if r[n] is None]
    if core:
        rep.add(FAIL, "classical_attitude の基本パラメータが読めない",
                f"{core} が未宣言。想定と違うノードが居る可能性がある")
        return r
    live = r["live_thrusters"]
    if dead is not None:
        want = [p not in dead for p in POSITIONS]
        if list(live) == want:
            rep.add(OK, "live_thrusters が control と一致 (B-16)", f"{dict(zip(POSITIONS, live))}")
        else:
            rep.add(FAIL, "live_thrusters が control と食い違う (B-16)",
                    f"autonomy {dict(zip(POSITIONS, live))} / control の esc_disabled から "
                    f"{dict(zip(POSITIONS, want))}\n死んだ基に配分し続ける (yaw の実現誤差 240%)")
    n_live = sum(1 for x in live if x)
    cap = float(r["max_duty"])
    if n_live < 4 and cap < 0.3:
        rep.add(WARN, f"{n_live} 基運用で cap が低い (B-18)",
                f"max_duty={cap:.2f}。3 基では rb/rf が 2〜3 割の時間 duty 飽和していた。"
                "0.3 から始めて安定したら 0.4")
    else:
        rep.add(OK, "max_duty", f"{cap:.2f} ({n_live} 基運用)")
    missing = [n for n in ("imu_timeout", "vel_timeout") if r[n] is None]
    if missing:
        rep.add(FAIL, "断の検出が入っていないバイナリ (B-19)",
                f"{missing} を宣言していない。**指令や IMU が途切れても走り続ける版。**\n"
                "Pi で git pull && colcon build すること")
        off = []
    else:
        off = [n for n in ("imu_timeout", "vel_timeout") if float(r[n]) <= 0.0]
    if off:
        rep.add(WARN, "断の検出が無効になっている (B-19)",
                f"{off} が 0。**指令や IMU が途切れても最後の値のまま走り続ける。**\n"
                "意図して切ったのでなければ 1.0 に戻すこと")
    elif not missing:
        rep.add(OK, "断の検出 (B-19)",
                f"imu_timeout={float(r['imu_timeout']):.1f}s / "
                f"vel_timeout={float(r['vel_timeout']):.1f}s")
    if not teleop:
        return r
    topic = str(r["cmd_target_topic"]).strip()
    mode = str(r["cmd_target_yaw_mode"]).strip().lower()
    if not topic:
        rep.add(FAIL, "UI のテレオペが姿勢制御に載っていない",
                "cmd_target_topic が空。`cmd_target_topic:=/cmd/target` を付けて起動すること")
    elif mode != "rate":
        rep.add(FAIL, f"cmd_target_yaw_mode={mode} (B-17)",
                "UI のスティック値が**絶対方位**として読まれるので ±11 度しか回れない。\n"
                f"`ros2 param set {ATTITUDE} cmd_target_yaw_mode rate`")
    else:
        rep.add(OK, "cmd_target_yaw_mode (B-17)", f"rate / topic={topic}")
    return r


def check_binary(rep, teleop):
    """**B-17 の修正が入ったバイナリか。** 入っていないと param set が黙って無視される。"""
    # symlink-install だと site-packages に実体が無い (egg-link) ので、**import して見る**。
    # ファイルを探しに行くと「見つからない」と「入っていない」が区別できなくなる
    try:
        import importlib

        mod = importlib.import_module("umiusi_autonomy.classical_attitude_node")
    except Exception as e:  # noqa: BLE001
        rep.add(WARN, "classical_attitude_node を import できない",
                f"{type(e).__name__}: {e} (source していない環境で実行している)")
        return
    where = getattr(mod, "__file__", "?")
    src = Path(where).read_text() if Path(where).exists() else ""
    fixed = hasattr(mod, "advance_yaw_setpoint") and '"cmd_target_yaw_mode"' in src
    level = OK if fixed else (FAIL if teleop else WARN)
    rep.add(level, "B-17 の修正が入っているか (B-17)",
            (f"入っている\n{where}" if fixed else
             f"**入っていない。** `ros2 param set` が成功を返すのに効かない版。\n"
             f"{where}\nPi で `git pull && colcon build --packages-select umiusi_autonomy "
             "--symlink-install` すること"))


def check_graph(rep, node):
    pubs = node.get_publishers_info_by_topic(CMD_TOPIC)
    names = sorted({p.node_name for p in pubs})
    if len(names) > 1:
        rep.add(FAIL, "/cmd/direct に publisher が 2 つ以上 (B-12)",
                f"{names}。古典と RL を同時に上げている。指令を取り合う")
    elif names:
        rep.add(OK, "/cmd/direct の publisher (B-12)", f"{names[0]} だけ")
    else:
        rep.add(WARN, "/cmd/direct に publisher が居ない",
                "姿勢制御器がまだ指令を出していない (arm 前なら正常)")
    if not any(t == PRESSURE for t, _ in node.get_topic_names_and_types()):
        rep.add(WARN, "深度センサが出ていない",
                f"{PRESSURE} が無い。**深度は閉ループにできない** "
                "(古典の heave は前進項のみ・k_v_vert 既定 0)")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--teleop", action="store_true", help="UI のゲームパッドで操縦する日")
    ap.add_argument("--bundle", default="", help="バンドルの場所 (既定は機体が使っているもの)")
    a = ap.parse_args()

    rclpy.init()
    node = Node("preflight")
    rep = Report()
    try:
        dead = check_is_forward(rep, node)
        r = check_attitude(rep, node, dead, a.teleop)
        path = a.bundle or (r and str(r["bundle_path"]).strip())
        if not path:
            from ament_index_python.packages import get_package_share_directory
            path = str(Path(get_package_share_directory("umiusi_autonomy"))
                       / "config" / "classical_bundle.json")
        check_bundle(rep, path)
        check_binary(rep, a.teleop)
        check_graph(rep, node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return rep.dump()


if __name__ == "__main__":
    sys.exit(main())
