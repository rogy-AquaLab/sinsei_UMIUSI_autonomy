#!/usr/bin/env python3
"""bag の /state/imu から「1 サンプルの大きな姿勢跳躍」を拾い、
それが yaw だけの跳躍か (= 磁気基準の再収束) それとも 3 軸の化けかを分解する。

出力: 各跳躍について 総角度 / roll,pitch,yaw の差分 / 回転軸の鉛直成分 / |q| / 同時刻の角速度。
回転軸が重力軸(z)にほぼ平行 = ヨーだけが飛んだ = 磁気。
"""
import math
import sys

from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import rosbag2_py

THRESH_DEG = 30.0
MAX_DT = 0.1   # s  これを超える間隔は「記録の欠落をまたいだ差分」なので跳躍として数えない
               # (9/13 の bag には 17〜25 秒の欠落があり、素朴に差分を取ると跳躍に見える)


def qnorm(q):
    return math.sqrt(sum(c * c for c in q))


def qmul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return (
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
    )


def qconj(q):
    w, x, y, z = q
    return (w, -x, -y, -z)


def to_rpy(q):
    w, x, y, z = q
    # ZYX (yaw-pitch-roll)
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    s = max(-1.0, min(1.0, 2 * (w * y - z * x)))
    pitch = math.asin(s)
    yaw = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return [math.degrees(v) for v in (roll, pitch, yaw)]


def wrap(d):
    while d > 180:
        d -= 360
    while d < -180:
        d += 360
    return d


def main(path, topic="/state/imu"):
    r = rosbag2_py.SequentialReader()
    r.open(
        rosbag2_py.StorageOptions(uri=path, storage_id=""),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {t.name: t.type for t in r.get_all_topics_and_types()}
    if topic not in types:
        print("topics:", sorted(types))
        return 1
    msgtype = get_message(types[topic])

    prev = None
    t0 = None
    n = 0
    gaps = 0
    hits = []
    while r.has_next():
        tname, data, ts = r.read_next()
        if tname != topic:
            continue
        m = deserialize_message(data, msgtype)
        o = m.orientation
        q = (o.w, o.x, o.y, o.z)
        g = m.angular_velocity
        nq = qnorm(q)
        n += 1
        if t0 is None:
            t0 = ts
        t = (ts - t0) / 1e9
        if abs(nq - 1.0) > 0.01:
            prev = None  # 化けは基準にしない
            continue
        q = tuple(c / nq for c in q)
        if prev is not None:
            pq, pt, prpy = prev
            rel = qmul(qconj(pq), q)
            if rel[0] < 0:
                rel = tuple(-c for c in rel)
            ang = math.degrees(2 * math.acos(max(-1.0, min(1.0, rel[0]))))
            if ang > THRESH_DEG and (t - pt) > MAX_DT:
                gaps += 1
            if ang > THRESH_DEG and (t - pt) <= MAX_DT:
                rpy = to_rpy(q)
                s = math.sqrt(max(0.0, 1 - rel[0] ** 2))
                axis = (
                    [c / s for c in rel[1:]] if s > 1e-9 else [0.0, 0.0, 0.0]
                )
                # 軸を body から world(重力基準) へ回す: v_w = q * v_b * q^-1
                vb = (0.0, axis[0], axis[1], axis[2])
                vw = qmul(qmul(q, vb), qconj(q))[1:]
                hits.append(
                    dict(
                        t=t,
                        dt=t - pt,
                        ang=ang,
                        drpy=[wrap(rpy[i] - prpy[i]) for i in range(3)],
                        axis_world_z=vw[2],
                        axis_world=vw,
                        nq=nq,
                        gyro=(g.x, g.y, g.z),
                    )
                )
            prev = (q, t, to_rpy(q))
        else:
            prev = (q, t, to_rpy(q))

    print(f"samples={n}  jumps(>{THRESH_DEG}deg, dt<={MAX_DT}s)={len(hits)}"
          f"  [記録の欠落をまたいだ差分 {gaps} 件は除外]")
    for h in sorted(hits, key=lambda h: -h["ang"])[:12]:
        dr, dp, dy = h["drpy"]
        gx, gy, gz = h["gyro"]
        print(
            f"t={h['t']:7.3f}s dt={h['dt']*1000:5.1f}ms  |ang|={h['ang']:7.2f}deg  "
            f"droll={dr:8.2f} dpitch={dp:8.2f} dyaw={dy:8.2f}  "
            f"axis_world=({h['axis_world'][0]:+.3f},{h['axis_world'][1]:+.3f},{h['axis_world'][2]:+.3f})  "
            f"|q|={h['nq']:.5f}  gyro=({gx:+.2f},{gy:+.2f},{gz:+.2f})"
        )
    return 0


if __name__ == "__main__":
    sys.exit(main(*sys.argv[1:]))
