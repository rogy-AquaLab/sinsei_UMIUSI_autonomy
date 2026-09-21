#!/usr/bin/env python3
"""bag から**推力の符号を軸ごとに測る**。専用の実験を回さなくてよい。

`thrust_sign_check.py` は 1 基ずつステップを入れる専用の実験が要る。こちらは
**姿勢制御を回した普通の bag**からそのまま測れるので、実機の時間を使わない。

## 原理

配分行列の構造上、2 つの群はトルクの作り先が重ならない:

  * 水平成分 (`thrust_axes`)  -> **yaw にしかトルクを作らない**
  * 垂直成分 (`_Y_UP`)        -> **roll / pitch にしかトルクを作らない**

なので yaw の係数が水平の符号を、roll/pitch の係数が垂直の符号を**独立に**決める。
実際に出した duty (`/state/thruster_state_all` のエコー) とサーボ角から予測トルクを作り、
実測角加速度へ回帰する:

    wdot = a*tau_pred + b*w + c*w|w| + d*sin(傾き) + e

**a の符号がその軸の推力の符号。** 正 = バンドルの規約どおり / 負 = 反転。

roll/pitch には浮力の復元が乗るが `d*sin(傾き)` として**回帰で分離する**。分離しないまま
「arm したら roll が落ち着いた」を符号の根拠にしてはいけない — 復元だけでも落ち着く。

## 出力の読みかた

**yaw を対照に使う。** 姿勢制御が方位を保てていた bag なら yaw の符号は合っているはずで、
そこが正に出ない設定は代理変数の選びかたが悪い。推力の代理変数を数通り振って、
yaw が正に出る組だけを信用する (`--all-proxies` で全部出す)。

R^2 は 0.1〜0.5 程度にしかならない (波・係留・推定していない並進の効き)。**見るのは
係数の符号と z 値**であって当てはまりの良さではない。

使いかた:
    python3 tools/thrust_sign_from_bag.py <bag-dir> [--bundle <classical_bundle.json>]
    python3 tools/thrust_sign_from_bag.py <bag-dir> --all-proxies

前提: `/state/thruster_state_all` と `/state/imu` が入っていること。arm 後だけを使う
(`/rosout` の `classical_attitude: ARMED`。無ければ bag 全体)。
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
from rcl_interfaces.msg import Log
from rclpy.serialization import deserialize_message
from rosbag2_py import ConverterOptions, SequentialReader, StorageOptions
from sensor_msgs.msg import Imu
from sinsei_umiusi_msgs.msg import ThrusterStateAll

POSITIONS = ("lf", "lb", "rb", "rf")
Y_UP = np.array([0.0, 1.0, 0.0])
STATE_TOPIC = "/state/thruster_state_all"
IMU_TOPIC = "/state/imu"


def rep103_from_cad(v):
    """CAD (+X 前, +Y 上, +Z 右舷) -> REP-103 (x 前, y 左, z 上)。"""
    return np.stack([v[..., 0], -v[..., 2], v[..., 1]], axis=-1)


def read(path):
    r = SequentialReader()
    r.open(StorageOptions(uri=str(path), storage_id="mcap"),
           ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"))
    st, imu, armed, t0 = [], [], None, None
    while r.has_next():
        topic, data, t = r.read_next()
        if t0 is None:
            t0 = t
        ts = (t - t0) / 1e9
        if topic == STATE_TOPIC:
            m = deserialize_message(data, ThrusterStateAll)
            row = [ts]
            for p in POSITIONS:
                s = getattr(m, p)
                row += [s.duty_cycle, s.angle, s.rpm]
            st.append(row)
        elif topic == IMU_TOPIC:
            m = deserialize_message(data, Imu)
            q, g = m.orientation, m.angular_velocity
            imu.append((ts, q.w, q.x, q.y, q.z, g.x, g.y, g.z))
        elif topic == "/rosout" and armed is None:
            m = deserialize_message(data, Log)
            if m.name == "classical_attitude" and m.msg.strip() == "ARMED":
                armed = ts
    return np.array(st), np.array(imu), armed


def lag1(x, t, tau):
    y = np.zeros_like(x)
    for i in range(1, len(x)):
        dt = min(max(t[i] - t[i - 1], 0.0), 0.2)
        y[i] = y[i - 1] + dt / tau * (x[i] - y[i - 1])
    return y


PROXIES = {
    # 推力 ∝ 指令^2 (bundle の thrust_curve_exp=2)。duty は 50 Hz で新しいのでこれが基準
    "duty^2": lambda duty, ang, rpm, t: np.sign(duty) * duty ** 2,
    "duty^2+lag0.3": lambda duty, ang, rpm, t: lag1(np.sign(duty) * duty ** 2, t, 0.3),
    "duty": lambda duty, ang, rpm, t: duty.copy(),
    # rpm は**実測**だがテレメトリが 2〜15 Hz しか更新されない (known_issues B-18)。
    # 大きさだけ使い、向きは新しい duty から取る
    "rpm^2": lambda duty, ang, rpm, t: np.sign(duty) * (rpm / 1000.0) ** 2,
}


def torque_of(st, axes, piv, proxy):
    tau = np.zeros((len(st), 3))
    for k in range(4):
        duty, ang, rpm = st[:, 1 + 3 * k], st[:, 2 + 3 * k], st[:, 3 + 3 * k]
        phi = np.radians(ang)
        thr = PROXIES[proxy](duty, ang, rpm, st[:, 0])
        dirv = (np.cos(phi)[:, None] * axes[k][None, :]
                + np.sin(phi)[:, None] * Y_UP[None, :])
        tau += np.cross(piv[k][None, :], thr[:, None] * dirv)
    return rep103_from_cad(tau)


def fit(st, imu, axes, piv, proxy):
    tau = torque_of(st, axes, piv, proxy)
    q = np.stack([np.interp(st[:, 0], imu[:, 0], imu[:, 1 + j]) for j in range(4)], axis=1)
    w = np.stack([np.interp(st[:, 0], imu[:, 0], imu[:, 5 + j]) for j in range(3)], axis=1)
    w_, x_, y_, z_ = q.T
    roll = np.arctan2(2 * (w_ * x_ + y_ * z_), 1 - 2 * (x_ * x_ + y_ * y_))
    pitch = np.arcsin(np.clip(2 * (w_ * y_ - z_ * x_), -1, 1))
    k = np.ones(9) / 9.0
    wd = np.stack([np.gradient(np.convolve(w[:, j], k, mode="same"), st[:, 0])
                   for j in range(3)], axis=1)
    out = {}
    for name, j, rest in (("roll", 0, np.sin(roll)), ("pitch", 1, np.sin(pitch)), ("yaw", 2, None)):
        cols = [tau[:, j], w[:, j], w[:, j] * np.abs(w[:, j])]
        if rest is not None:
            cols.append(rest)
        cols.append(np.ones(len(st)))
        A = np.vstack(cols).T
        g = np.all(np.isfinite(A), axis=1) & np.isfinite(wd[:, j])
        if g.sum() < 200:
            continue
        sol, *_ = np.linalg.lstsq(A[g], wd[g, j], rcond=None)
        res = wd[g, j] - A[g] @ sol
        ss = ((wd[g, j] - wd[g, j].mean()) ** 2).sum()
        cov = np.linalg.pinv(A[g].T @ A[g]) * (res @ res) / (g.sum() - len(sol))
        se = float(np.sqrt(np.diag(cov))[0])
        out[name] = (float(sol[0]), se, 1 - (res @ res) / ss,
                     float(sol[3]) if rest is not None else float("nan"))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag")
    ap.add_argument("--bundle", default="", help="classical_bundle.json (既定は同梱のもの)")
    ap.add_argument("--all-proxies", action="store_true", help="推力の代理変数を全部試す")
    ap.add_argument("--skip-sec", type=float, default=5.0, help="arm 直後に捨てる秒数")
    a = ap.parse_args()

    path = a.bundle
    if not path:
        from ament_index_python.packages import get_package_share_directory
        path = Path(get_package_share_directory("umiusi_autonomy")) / "config" / "classical_bundle.json"
    c = json.loads(Path(path).read_text())["contract"]
    axes, piv = np.asarray(c["thrust_axes"], float), np.asarray(c["pivots_from_com"], float)

    st, imu, armed = read(a.bag)
    if not len(st) or not len(imu):
        print(f"{STATE_TOPIC} か {IMU_TOPIC} が bag に無い", file=sys.stderr)
        return 1
    t_arm = (armed or 0.0) + a.skip_sec
    st = st[st[:, 0] > t_arm]
    if len(st) < 200:
        print(f"arm 後のサンプルが {len(st)} 件しかない (t_arm={t_arm:.1f}s)。"
              "--skip-sec を減らすか、arm 後に長く回した bag を使うこと", file=sys.stderr)
        return 1
    print(f"bundle: {path}")
    print(f"arm={armed if armed is not None else '不明 (bag 全体を使う)'}  "
          f"使うサンプル {len(st)} 件 ({st[-1, 0] - st[0, 0]:.0f}s)")

    proxies = list(PROXIES) if a.all_proxies else ["duty^2+lag0.3", "duty^2", "rpm^2"]
    print("\n  代理変数        軸      a (推力の符号)        z      R^2   復元 d")
    verdict = {}
    for pr in proxies:
        r = fit(st, imu, axes, piv, pr)
        for ax in ("yaw", "roll", "pitch"):
            if ax not in r:
                continue
            v, se, r2, d = r[ax]
            z = v / se if se > 0 else 0.0
            print(f"  {pr:14s} {ax:6s} {v:+12.3f} {z:+9.1f} {r2:8.2f} {d:+8.2f}")
            verdict.setdefault(ax, []).append(z)
        print()

    print("判定 (** yaw を対照に使うこと **):")
    for ax in ("yaw", "roll", "pitch"):
        zs = verdict.get(ax, [])
        if not zs:
            continue
        pos = sum(1 for z in zs if z > 3)
        neg = sum(1 for z in zs if z < -3)
        s = "規約どおり" if pos and not neg else ("**反転**" if neg and not pos else "割れている")
        group = "水平 (thrust_axes)" if ax == "yaw" else "垂直 (_Y_UP / duty の符号)"
        print(f"  {ax:6s} [{group}] {s}  (正 {pos} / 負 {neg} / 全 {len(zs)} 通り)")
    print("\n  yaw が正に出ない場合、その bag では姿勢制御が方位を保てていないか、"
          "代理変数が合っていない。roll/pitch の判定も信用しないこと。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
