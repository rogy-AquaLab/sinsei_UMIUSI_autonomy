"""旋回レート指令を方位目標へ積分する。**navigator と姿勢制御器で同じものを使う。**

同じ式が 2 箇所にあると片方だけ直る。実際、`classical_attitude_node` の
`cmd_target_yaw_mode="rate"` と `navigator_node` の setpoint 経路は**同一のアルゴリズムを
別々に書いていた**。known_issues B-17 は前者の `dt` の取り違えで、後者は無傷だった —
次に同じことが起きる保証は無い。
"""
from __future__ import annotations

import math

# 指令が途切れたときに方位目標が飛ばないための上限 [s]。
# 1 ステップぶんに抑える (止まっていた時間をまとめて積むと目標が飛ぶ)
YAW_DT_MAX = 0.2


def wrap(a):
    """角度を (-pi, pi] に畳む。"""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def advance_yaw_setpoint(yaw_sp, yaw_now, rz, rate_scale, dt, lead_max):
    """旋回レート指令 `rz` を方位目標へ積分する。

    呼び出し側の義務:
      * `dt` = **前の指令から今までの実時間**。制御周期を渡すと旋回速度が指令の
        送信レートに比例してずれる (known_issues B-17)
      * `lead_max` = 実測方位からの先行量の上限 (known_issues B-14 の 180 度の罠)。
        機体が追随できない目標を積み続けると誤差が 180 度に達し、**そこが安定平衡になって
        出られなくなる**
      * `yaw_sp is None` なら**実測方位から始める**。0 (= IMU の基準方位) から始めると
        現場の向き次第でいきなり大きな誤差になる
    """
    if yaw_sp is None:
        yaw_sp = yaw_now
    yaw_sp = wrap(yaw_sp + rz * rate_scale * dt)
    lead = wrap(yaw_sp - yaw_now)
    if abs(lead) > lead_max:      # 追随できない目標を先行させない
        yaw_sp = wrap(yaw_now + math.copysign(lead_max, lead))
    return yaw_sp
