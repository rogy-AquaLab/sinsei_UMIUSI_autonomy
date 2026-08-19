#!/usr/bin/env python3
"""傾け→水平復帰の軌跡を 50Hz 全数記録し、往復の一致度を評価する。"""
from __future__ import annotations
import math, sys, time
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu

def rpy(o):
    x, y, z, w = o.x, o.y, o.z, o.w
    n = math.sqrt(x*x+y*y+z*z+w*w)
    if n < 0.5:          # ゼロクォータニオン=データ化け
        return None
    x, y, z, w = x/n, y/n, z/n, w/n
    r = math.atan2(2*(w*x+y*z), 1-2*(x*x+y*y))
    s = 2*(w*y-z*x); s = max(-1.0, min(1.0, s))
    p = math.asin(s)
    yw = math.atan2(2*(w*z+x*y), 1-2*(y*y+z*z))
    return math.degrees(r), math.degrees(p), math.degrees(yw)

class T(Node):
    """動きを検知するまで待機し、検知後 dur 秒だけ記録する(自動トリガ)。"""
    def __init__(self, dur):
        super().__init__("imu_trace")
        self.armed=False; self.base=None; self.wait0=time.time()
        self.create_subscription(Imu, "/state/imu",  self.cb,
            QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST, depth=1))
        self.rows=[]; self.bad=0; self.t0=time.time(); self.dur=dur
        self.create_timer(1.0, self.tick)
    def cb(self, m):
        v = rpy(m.orientation)
        if v is None: self.bad += 1; return
        g = m.angular_velocity
        if not self.armed:
            if self.base is None: self.base = v
            moved = (abs(v[0]-self.base[0])>4.0 or abs(v[1]-self.base[1])>4.0
                     or max(abs(g.x),abs(g.y),abs(g.z))>0.25)
            if moved:
                self.armed=True; self.t0=time.time()
                print(f"\n>>> 動きを検知しました。ここから {self.dur:.0f} 秒記録します <<<\n", flush=True)
                # 検知直前の基準として base を1点入れておく
                self.rows.append((0.0,)+self.base)
            return
        self.rows.append((time.time()-self.t0,)+v)
    def tick(self):
        if not self.armed:
            w=time.time()-self.wait0
            print(f"[待機 {w:4.0f}s] 傾けてください (基準 roll={self.base[0]:+6.2f} pitch={self.base[1]:+6.2f})"
                  if self.base else "[待機] /state/imu 待ち", flush=True)
            if w>420: print("タイムアウト"); raise SystemExit(0)
            return
        el=time.time()-self.t0
        if self.rows:
            _,r,p,y=self.rows[-1]
            print(f"[{el:5.1f}s] roll={r:+7.2f} pitch={p:+7.2f} yaw={y:+7.2f}", flush=True)
        if el>=self.dur: self.report(); raise SystemExit(0)
    def report(self):
        R=self.rows
        if not R: print("データなし"); return
        print("\n"+"="*66)
        print(f"傾け→復帰トレース  {len(R)} サンプル / {self.dur:.0f}秒  (化けサンプル {self.bad} 件除外)")
        print("="*66)
        base=R[:max(1,min(len(R),150))]    # 最初の3秒 = 基準水平
        br=sum(b[1] for b in base)/len(base); bp=sum(b[2] for b in base)/len(base)
        print(f"  基準(開始3秒平均):  roll={br:+7.2f}  pitch={bp:+7.2f}")
        tail=R[-150:]                       # 最後の3秒 = 復帰後
        er=sum(b[1] for b in tail)/len(tail); ep=sum(b[2] for b in tail)/len(tail)
        print(f"  復帰(終了3秒平均):  roll={er:+7.2f}  pitch={ep:+7.2f}")
        print(f"  ★ 復帰誤差:        roll={er-br:+7.2f} deg   pitch={ep-bp:+7.2f} deg")
        rv=[b[1] for b in R]; pv=[b[2] for b in R]
        print(f"  傾けた範囲:        roll {min(rv):+7.2f}..{max(rv):+7.2f}   pitch {min(pv):+7.2f}..{max(pv):+7.2f}")
        print("\n  --- 軌跡 (0.5秒ごと) ---")
        last=-9
        for t,r,p,y in R:
            if t-last>=0.5:
                last=t
                br_=int(max(-1,min(1,r/60))*20)
                bar="".join("#" if i==br_+20 else ("|" if i==20 else "-") for i in range(41))
                print(f"   {t:5.1f}s r={r:+7.2f} p={p:+7.2f} [{bar}]")
def main():
    d=float(sys.argv[1]) if len(sys.argv)>1 else 60.0
    rclpy.init(); n=T(d)
    try: rclpy.spin(n)
    except (KeyboardInterrupt, ExternalShutdownException, SystemExit): pass
    finally:
        n.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
if __name__=="__main__": main()
