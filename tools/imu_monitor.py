#!/usr/bin/env python3
"""実機 BNO055 の姿勢をライブ表示する。機体を傾けながら出力を目視確認するためのツール。

  ros2 launch sinsei_umiusi_control main.yaml enable_cameras:=false   # 別ターミナルで
  python3 imu_monitor.py

roll/pitch/yaw [deg] と角速度 [rad/s] を毎秒更新し、傾けた範囲(min/max)も追跡する。
"""
from __future__ import annotations

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu


def quat_to_rpy(x, y, z, w):
    """クォータニオン -> roll/pitch/yaw [rad] (ZYX順)"""
    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = math.copysign(math.pi / 2, sinp) if abs(sinp) >= 1 else math.asin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


def bar(val, lo=-90.0, hi=90.0, width=31):
    """-90..+90 deg を ASCII バーで表す（中央が 0）"""
    v = max(lo, min(hi, val))
    pos = int((v - lo) / (hi - lo) * (width - 1))
    cells = ["-"] * width
    cells[width // 2] = "|"
    cells[pos] = "#"
    return "".join(cells)


class ImuMonitor(Node):
    def __init__(self):
        super().__init__("imu_monitor")
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.sub = self.create_subscription(Imu, "/state/imu", self.cb, qos)
        self.n = 0
        self.last = None
        self.mm = {"roll": [999.0, -999.0], "pitch": [999.0, -999.0], "yaw": [999.0, -999.0]}
        self.create_timer(0.5, self.show)
        print("IMU モニタ開始 — 機体を傾けてください (Ctrl-C で終了)\n")

    def cb(self, msg: Imu):
        self.n += 1
        self.last = msg

    def show(self):
        if self.last is None:
            print("\r/state/imu 待機中... (control スタックは起動していますか?)", end="", flush=True)
            return
        m = self.last
        o = m.orientation
        r, p, y = (math.degrees(v) for v in quat_to_rpy(o.x, o.y, o.z, o.w))
        for k, v in (("roll", r), ("pitch", p), ("yaw", y)):
            self.mm[k][0] = min(self.mm[k][0], v)
            self.mm[k][1] = max(self.mm[k][1], v)
        g = m.angular_velocity
        a = m.linear_acceleration

        print("\033[2J\033[H", end="")   # 画面クリア
        print(f"=== BNO055 姿勢モニタ ===   受信 {self.n} msg\n")
        for k, v in (("roll ", r), ("pitch", p), ("yaw  ", y)):
            key = k.strip()
            lo, hi = self.mm[key]
            print(f"  {k} {v:+8.2f} deg  [{bar(v)}]   範囲 {lo:+7.1f} .. {hi:+7.1f}")
        print(f"\n  角速度 [rad/s]  x={g.x:+7.4f}  y={g.y:+7.4f}  z={g.z:+7.4f}")
        print(f"  加速度 [m/s^2]  x={a.x:+7.3f}  y={a.y:+7.3f}  z={a.z:+7.3f}")
        print(f"  クォータニオン  x={o.x:+.5f} y={o.y:+.5f} z={o.z:+.5f} w={o.w:+.5f}")
        print("\n  (Ctrl-C で終了)", flush=True)


def main():
    rclpy.init()
    node = ImuMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        print("\n終了")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
