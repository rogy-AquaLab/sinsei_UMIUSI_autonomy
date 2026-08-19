#!/usr/bin/env python3
"""静止状態の /state/imu を記録し、異常サンプル(グリッチ)の発生率を調べる。"""
from __future__ import annotations
import math, sys, time
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu

GYRO_FS = 35.74      # BNO055 int16 フルスケール [rad/s] (2047.9 deg/s)
GLITCH_TH = 5.0      # 静止中にこれを超えたら異常 [rad/s]

class G(Node):
    def __init__(self, dur):
        super().__init__("imu_glitch")
        qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST, depth=1)
        self.create_subscription(Imu, "/state/imu", self.cb, qos)
        self.n = 0; self.glitch = []; self.quat_bad = []
        self.gmax = [0.0, 0.0, 0.0]
        self.t0 = time.time(); self.dur = dur
        self.create_timer(1.0, self.tick)

    def cb(self, m):
        self.n += 1
        t = time.time() - self.t0
        g = (m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z)
        for i, v in enumerate(g):
            self.gmax[i] = max(self.gmax[i], abs(v))
        if any(abs(v) > GLITCH_TH for v in g):
            self.glitch.append((t, g))
        o = m.orientation
        norm = math.sqrt(o.x**2 + o.y**2 + o.z**2 + o.w**2)
        if abs(norm - 1.0) > 0.01:
            self.quat_bad.append((t, norm))

    def tick(self):
        if time.time() - self.t0 >= self.dur:
            self.report(); raise SystemExit(0)

    def report(self):
        d = time.time() - self.t0
        print("\n" + "="*64)
        print(f"静止時 IMU データ健全性   {self.n} サンプル / {d:.0f}秒 ({self.n/d:.1f} Hz)")
        print("="*64)
        print(f"  角速度の絶対値 最大: x={self.gmax[0]:.4f}  y={self.gmax[1]:.4f}  z={self.gmax[2]:.4f} rad/s")
        print(f"  グリッチ (|w| > {GLITCH_TH} rad/s): {len(self.glitch)} 件"
              f"  = {100*len(self.glitch)/max(1,self.n):.3f}%  ({len(self.glitch)/d:.2f} 件/秒)")
        for t, g in self.glitch[:8]:
            sat = "  <- フルスケール飽和" if any(abs(v) > GYRO_FS*0.95 for v in g) else ""
            print(f"    t={t:6.2f}s  x={g[0]:+9.4f} y={g[1]:+9.4f} z={g[2]:+9.4f}{sat}")
        if len(self.glitch) > 8:
            print(f"    ... 他 {len(self.glitch)-8} 件")
        print(f"  クォータニオン ノルム異常: {len(self.quat_bad)} 件")
        for t, nv in self.quat_bad[:5]:
            print(f"    t={t:6.2f}s  |q|={nv:.4f}")

def main():
    dur = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
    rclpy.init(); n = G(dur)
    try: rclpy.spin(n)
    except (KeyboardInterrupt, ExternalShutdownException, SystemExit): pass
    finally:
        n.destroy_node()
        if rclpy.ok(): rclpy.shutdown()

if __name__ == "__main__":
    main()
