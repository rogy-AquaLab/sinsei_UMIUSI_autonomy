#!/usr/bin/env python3
"""rl_attitude_node の目標姿勢を roll/pitch/yaw [deg] で与える。

素の設定は `~/setpoint` に AttitudeTarget (クォータニオン) を publish する形だが、
実験中に手で組み立てるのは面倒なので、度で指定できるようにしたもの。

    python3 set_attitude.py --yaw 90            # 右に 90 度向く指令
    python3 set_attitude.py --roll 20 --hold    # 指定して押し続ける (1 発だと取りこぼす)
    python3 set_attitude.py --level             # 水平・停止に戻す
    python3 set_attitude.py --vel 0.3           # 前進速度だけ変える
    python3 set_attitude.py --vel 0 0 -0.2 --hold   # 純下降 (3-D ポリシー限定。手動降下バースト)

`--hold` は 10 Hz で publish し続ける (Ctrl-C で停止)。QoS の depth が 1 なので、
起動直後などは 1 発だと届かないことがある。**実験では --hold を推奨**。

`--vel` を指定しないときは速度指令に触らない (IGNORE_VELOCITY)。姿勢だけ変えたつもりで
launch の `vel_cmd` (既定 0 = 姿勢保持) を黙って書き換えるのを避けるため。止めたいときは
`--level` か `--vel 0`。
"""
from __future__ import annotations

import argparse
import math

import rclpy
from rclpy.node import Node

from umiusi_rl_control_msgs.msg import AttitudeTarget


def rpy_to_quat(roll_deg: float, pitch_deg: float, yaw_deg: float):
    """roll/pitch/yaw [deg] -> (x, y, z, w)。ZYX 順で、IMU モニタの表示と合わせてある。"""
    r, p, y = (math.radians(v) / 2.0 for v in (roll_deg, pitch_deg, yaw_deg))
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return (sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy,
            cr * cp * cy + sr * sp * sy)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--roll", type=float, default=0.0, help="[deg]")
    ap.add_argument("--pitch", type=float, default=0.0, help="[deg]")
    ap.add_argument("--yaw", type=float, default=0.0, help="[deg]")
    # 1 値 = 前進のみ (従来どおり)。3 値 = REP-103 body [x前, y左, z上] の速度指令 —
    # 例: --vel 0 0 -0.2 は純下降 (深度センサなしで降下バーストを手動試験するときに使う。
    # 3-D ポリシー av_cal5_3d_rep103 限定。水平ポリシーに鉛直を入れると姿勢が崩壊する)
    ap.add_argument("--vel", type=float, nargs="+", default=None,
                    help="速度指令 [m/s]: 1 値=前進のみ / 3 値=body [x y z] (未指定なら変更しない)")
    ap.add_argument("--level", action="store_true", help="水平・停止に戻す")
    ap.add_argument("--attitude-only", action="store_true", help="速度指令は無視させる")
    ap.add_argument("--topic", default="/rl_attitude_node/setpoint")
    ap.add_argument("--hold", action="store_true", help="10 Hz で publish し続ける")
    a = ap.parse_args()

    if a.level:
        a.roll = a.pitch = a.yaw = 0.0
        a.vel = [0.0]
    if a.vel is not None and len(a.vel) not in (1, 3):
        ap.error("--vel は 1 値 (前進) か 3 値 (body x y z)")

    msg = AttitudeTarget()
    x, y, z, w = rpy_to_quat(a.roll, a.pitch, a.yaw)
    msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = x, y, z, w
    # --vel 未指定は「速度を変更しない」= IGNORE_VELOCITY。0 を送ると last-wins で
    # launch の vel_cmd を黙って止めてしまう。--level は vel=0 を明示するので送る側。
    keep_vel = a.vel is None or a.attitude_only
    if a.vel is not None:
        v = a.vel if len(a.vel) == 3 else [a.vel[0], 0.0, 0.0]
        msg.velocity.x, msg.velocity.y, msg.velocity.z = (float(c) for c in v)
    msg.type_mask = AttitudeTarget.IGNORE_VELOCITY if keep_vel else 0

    rclpy.init()
    node = Node("set_attitude")
    pub = node.create_publisher(AttitudeTarget, a.topic, 1)

    vel_txt = ("速度=変更しない" if keep_vel else
               f"速度=[{msg.velocity.x:.2f},{msg.velocity.y:.2f},{msg.velocity.z:.2f}] m/s")
    print(f"目標: roll={a.roll:+.1f} pitch={a.pitch:+.1f} yaw={a.yaw:+.1f} deg"
          f"  {vel_txt}"
          f"{' (姿勢のみ)' if a.attitude_only else ''}  -> {a.topic}")

    try:
        if a.hold:
            print("  10 Hz で送り続けます (Ctrl-C で停止)")
            timer_period = 0.1
            def tick():
                msg.header.stamp = node.get_clock().now().to_msg()
                pub.publish(msg)
            node.create_timer(timer_period, tick)
            rclpy.spin(node)
        else:
            # 購読側が繋がるまで少し待ってから数発送る (depth=1 の取りこぼし対策)
            for _ in range(20):
                rclpy.spin_once(node, timeout_sec=0.05)
                if pub.get_subscription_count() > 0:
                    break
            if pub.get_subscription_count() == 0:
                print("  ⚠ 購読者が居ません (rl_attitude_node は起動していますか?)")
            for _ in range(3):
                msg.header.stamp = node.get_clock().now().to_msg()
                pub.publish(msg)
                rclpy.spin_once(node, timeout_sec=0.05)
            print("  送信しました")
    except KeyboardInterrupt:
        print("\n停止")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
