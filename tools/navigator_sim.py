#!/usr/bin/env python3
"""navigator の setpoint 経路を**実機に合わせた偽機体**で閉ループに回して詰める。

## なぜ MuJoCo を使わないか

MuJoCo の plant は `configs/umiusi.yaml` の `thrust_axes` を使っており、**2026-09-22 時点で
実機と yaw が鏡像**（sim セッションが反転作業中）。その上で詰めた旋回のゲインは実機で
逆に効くので使えない。ここでは**配備バンドルの幾何をそのまま真値**にする。

## 何を真値にしているか

  * 幾何・質量・抗力・付加質量 : `classical_bundle.json` の contract（= 実機に配る値）
  * roll / pitch の復元        : **9/13 の実機 bag から回帰で出した値**
                                 (roll -1.3, pitch -0.53 rad/s^2 per rad。known_issues B-18)
  * スラスタの起動の死に時間   : **実測 2.7 s**（duty 0.15 のステップ。B-18）。
                                 `--dead-time 0` で切れる
  * 1 基欠損                   : `--live` で指定（既定は 4 基）

**較正されていない値**（`thrust_per_cmd` / `drag_*` / `added_mass_*`）はバンドルのまま。
絶対値は信用できないので、**ゲインの相対比較に使う**こと。

## 使いかた

    python3 tools/navigator_sim.py                      # 既定値で 1 本流す
    python3 tools/navigator_sim.py --sweep yaw_rate     # 旋回ゲインを振る
    python3 tools/navigator_sim.py --sweep lead_max --dead-time 0
    python3 tools/navigator_sim.py --scenario approach  # 風船へ寄る想定の指令列

前提: `umiusi_perception` が import できること。
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "umiusi_common"))
from umiusi_common.yaw_setpoint import advance_yaw_setpoint  # noqa: E402

Y_UP = np.array([0.0, 1.0, 0.0])
# 9/13 の実機 bag から回帰で出した復元の強さ [rad/s^2 per rad] (known_issues B-18)。
# 自励周期は roll 約 5 s / pitch 約 9 s。**sim の値ではなく実機の値**
RESTORE_ROLL, RESTORE_PITCH = -1.3, -0.53
DEAD_TIME = 2.7          # [s] 静止からの起動の死に時間 (duty 0.15 のステップで実測)


def rpy_of(q):
    w, x, y, z = q
    return (math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y)),
            math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x)))),
            math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def quat_of_yaw(yaw):
    return np.array([math.cos(yaw / 2.0), 0.0, 0.0, math.sin(yaw / 2.0)])


class FakeVehicle:
    """6 自由度の剛体。**バンドルの規約で力を出す** = 実機・control と同じ向き。"""

    def __init__(self, contract, dt, dead_time=DEAD_TIME, live=(1, 1, 1, 1)):
        self.axes = np.asarray(contract["thrust_axes"], float)
        self.piv = np.asarray(contract["pivots_from_com"], float)
        self.m = float(contract["mass"]) + np.asarray(contract["added_mass_diag"], float)
        self.lin = np.asarray(contract["drag_lin"], float)
        self.quad = np.asarray(contract["drag_quad"], float)
        self.thrust_per_cmd = float(contract["thrust_per_cmd"])
        self.exp = float(contract["thrust_curve_exp"])
        self.buoy = float(contract["net_buoy_up"])
        self.dt, self.dead, self.live = dt, dead_time, np.asarray(live, float)
        # 惰性で回り続ける時間 [s]。**停止も実測で約 2.9 s かかる** ので、duty が一瞬
        # ゼロを横切っても止まらない。ここを入れないと死に時間を過大に見積もる
        self.coast = 2.9
        # 回転の慣性は契約に無いので、幾何から概算する (相対比較にしか使わない)
        self.inertia = np.array([0.35, 0.35, 0.55])
        self.reset()

    def reset(self):
        self.v = np.zeros(3)         # CAD 系の速度
        self.w = np.zeros(3)         # REP-103 の角速度 (roll, pitch, yaw)
        self.rpy = np.zeros(3)
        self._spun = np.zeros(4)     # 指令が乗っている連続時間 [s] (起動の判定用)
        self._idle = np.full(4, 1e9)  # 指令が切れてからの時間 [s] (惰性の判定用)
        self._live_spin = np.zeros(4, bool)   # 実際に回っているか

    @property
    def quat(self):
        return quat_of_yaw(self.rpy[2])   # roll/pitch は小さいので方位だけ持つ

    def step(self, action):
        """action = [servo x4 (±1), duty x4]。実機の死に時間を入れて力にする。"""
        servo = np.asarray(action[:4], float) * (math.pi / 2.0)
        duty = np.asarray(action[4:], float) * self.live
        # **起動の死に時間**: 止まっていた基は dead 秒ぶん推力が出ない (実機で実測)。
        # ただし**一度回れば惰性で coast 秒は回り続ける**ので、ゼロ交差では止まらない
        cmdd = np.abs(duty) > 0.01
        self._idle = np.where(cmdd, 0.0, self._idle + self.dt)
        self._spun = np.where(cmdd, self._spun + self.dt, self._spun)
        stopped = self._idle > self.coast
        self._spun = np.where(stopped, 0.0, self._spun)
        self._live_spin = np.where(
            stopped, False,
            self._live_spin | (self._spun >= self.dead) | (self.dead <= 0.0))
        thrust = (np.sign(duty) * np.abs(duty) ** self.exp
                  * self.thrust_per_cmd * self._live_spin)
        f = np.zeros(3)
        tau = np.zeros(3)
        for k in range(4):
            d = math.cos(servo[k]) * self.axes[k] + math.sin(servo[k]) * Y_UP
            f += thrust[k] * d
            tau += np.cross(self.piv[k], thrust[k] * d)
        f[1] += self.buoy
        drag = self.lin * self.v + self.quad * np.abs(self.v) * self.v
        self.v += (f - drag) / self.m * self.dt
        # トルクは CAD -> REP-103 (x 前, y 左, z 上)
        t = np.array([tau[0], -tau[2], tau[1]])
        t[0] += RESTORE_ROLL * self.inertia[0] * math.sin(self.rpy[0])
        t[1] += RESTORE_PITCH * self.inertia[1] * math.sin(self.rpy[1])
        t -= 0.8 * self.w * self.inertia           # 回転の抗力 (概算)
        self.w += t / self.inertia * self.dt
        self.rpy += self.w * self.dt
        self.rpy[2] = (self.rpy[2] + math.pi) % (2 * math.pi) - math.pi
        return self.v.copy()


def run(bundle, cmds, yaw_rate=0.6, lead_max=1.05, surge=0.35, cap=0.25,
        hz=50.0, dead_time=DEAD_TIME, live=(1, 1, 1, 1)):
    """FSM の {surge, heave, yaw} 列を流し、navigator -> 姿勢制御器 -> 偽機体を回す。"""
    from umiusi_perception.classical import (ClassicalController, GeneralAllocator,
                                             PlantContract, cad_wrench_from_modes,
                                             rep103_from_cad)
    b = json.loads(Path(bundle).read_text())
    plant = PlantContract.from_dict(b["contract"])
    ctl = ClassicalController(plant, **b.get("gains", {}))
    alloc = GeneralAllocator(plant, **b.get("allocator", {}))
    alloc.set_live(np.asarray(live, bool))
    veh = FakeVehicle(b["contract"], 1.0 / hz, dead_time, live)
    dt = 1.0 / hz
    yaw_sp, action, log = None, np.zeros(8), []
    for cmd in cmds:
        yaw_now = veh.rpy[2]
        yaw_sp = advance_yaw_setpoint(yaw_sp, yaw_now, cmd["yaw"], yaw_rate, dt, lead_max)
        target = quat_of_yaw(yaw_sp)
        vel_cmd = np.array([cmd["surge"] * surge, 0.0, cmd["heave"] * surge])
        # 姿勢制御器と同じ順序で回す
        from umiusi_autonomy.classical_attitude_node import sub_quat
        ori_err = sub_quat(target, veh.quat)
        v_hat = ctl.obs.update(action, veh.quat, dt)
        modes = ctl.wrench(ori_err, veh.w, vel_cmd, rep103_from_cad(v_hat), cap)
        wrench = cad_wrench_from_modes(modes, ctl.f_max_total(ctl.cap))
        action = alloc.allocate(wrench, cap)
        action = np.concatenate([action[:4], np.clip(action[4:], -cap, cap)])
        veh.step(action)
        log.append((yaw_sp, veh.rpy[2], veh.v[0], veh.rpy[0], veh.rpy[1],
                    float(np.max(np.abs(action[4:])))))
    return np.array(log)


def scenario(name, secs, hz=50.0):
    """FSM が出しそうな指令列。**本物の FSM ではなく、詰めるための代表パターン。**"""
    n = int(secs * hz)
    if name == "search":       # その場旋回 (SEARCH_YAW=0.5 相当)
        return [{"surge": 0.0, "heave": 0.0, "yaw": 0.5} for _ in range(n)]
    if name == "approach":     # 前進しながら 20 秒かけて方位を寄せる
        return [{"surge": 0.6, "heave": 0.0,
                 "yaw": 0.5 if i < n * 0.4 else 0.0} for i in range(n)]
    if name == "step":         # 方位をステップで振る (整定を見る)
        return [{"surge": 0.0, "heave": 0.0,
                 "yaw": 1.0 if i < hz * 3 else 0.0} for i in range(n)]
    if name.startswith("pulse"):
        # **短い旋回パルスを繰り返す。** FSM が「少し向きを直す」ときの形。
        # 起動の死に時間があるので、パルスが短いほど**指令の大半が推力にならない**
        w = float(name[5:] or 1.0)          # パルス幅 [s]
        period = w + 2.0
        return [{"surge": 0.0, "heave": 0.0,
                 "yaw": 1.0 if (i / hz) % period < w else 0.0} for i in range(n)]
    raise SystemExit(f"不明なシナリオ: {name}")


def summarise(log, cap, hz=50.0):
    sp, yaw, vx, roll, pitch, duty = log.T
    err = np.degrees(np.abs((sp - yaw + math.pi) % (2 * math.pi) - math.pi))
    turned = np.degrees(np.unwrap(yaw)[-1] - np.unwrap(yaw)[0])
    tail = slice(int(len(log) * 0.6), None)
    return {
        "旋回量[deg]": turned,
        "旋回レート[deg/s]": turned / (len(log) / hz),
        "方位誤差 p95[deg]": float(np.percentile(err, 95)),
        "方位誤差 末尾中央[deg]": float(np.median(err[tail])),
        "前進速度 末尾[m/s]": float(np.median(vx[tail])),
        "|roll| p95[deg]": float(np.percentile(np.degrees(np.abs(roll)), 95)),
        # **cap と比べる。** 「自分のピークに居た割合」を数えると、duty が一定なだけで
        # 100% になって意味が無い (最初の版がそうなっていた)
        "duty 飽和率[%]": float(np.mean(duty >= 0.99 * cap) * 100),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--bundle", default="")
    ap.add_argument("--scenario", default="search",
                help="search | approach | step | pulse<秒> (例 pulse0.5)")
    ap.add_argument("--secs", type=float, default=30.0)
    ap.add_argument("--yaw-rate", type=float, default=0.6)
    ap.add_argument("--lead-max", type=float, default=1.05)
    ap.add_argument("--surge", type=float, default=0.35)
    ap.add_argument("--cap", type=float, default=0.25)
    ap.add_argument("--dead-time", type=float, default=DEAD_TIME)
    ap.add_argument("--live", default="1,1,1,1", help="生きているスラスタ (lf,lb,rb,rf)")
    ap.add_argument("--sweep", default="", choices=("", "yaw_rate", "lead_max", "surge", "cap"))
    a = ap.parse_args()

    path = a.bundle
    if not path:
        from ament_index_python.packages import get_package_share_directory
        path = str(Path(get_package_share_directory("umiusi_autonomy"))
                   / "config" / "classical_bundle.json")
    live = tuple(int(x) for x in a.live.split(","))
    cmds = scenario(a.scenario, a.secs)
    base = dict(yaw_rate=a.yaw_rate, lead_max=a.lead_max, surge=a.surge, cap=a.cap,
                dead_time=a.dead_time, live=live)
    grids = {"yaw_rate": [0.2, 0.4, 0.6, 0.9, 1.2], "lead_max": [0.35, 0.52, 1.05, 1.57],
             "surge": [0.15, 0.25, 0.35, 0.5], "cap": [0.25, 0.3, 0.4]}

    print(f"# シナリオ {a.scenario} / {a.secs:.0f}s / 死に時間 {a.dead_time:.1f}s / live={live}")
    print(f"# bundle {path}")
    keys = None
    for val in (grids[a.sweep] if a.sweep else [None]):
        kw = dict(base)
        if a.sweep:
            kw[a.sweep] = val
        r = summarise(run(path, cmds, **kw), kw["cap"])
        if keys is None:
            keys = list(r)
            head = f"{a.sweep:>10s}" if a.sweep else f"{'':>10s}"
            print(head + "".join(f"{k:>20s}" for k in keys))
        lead = f"{val:>10.2f}" if a.sweep else f"{'既定':>10s}"
        print(lead + "".join(f"{r[k]:>20.2f}" for k in keys))
    return 0


if __name__ == "__main__":
    sys.exit(main())
