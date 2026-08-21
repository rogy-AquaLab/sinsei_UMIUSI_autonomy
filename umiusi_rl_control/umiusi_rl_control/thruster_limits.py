"""スラスタ指令の制限。ROS 非依存なので単体でテストできる。

**sim (`umiusi_sim/physics/thruster.py`) と同じレート制限を実機側にも掛けるためのもの。**

sim はポリシーの指令をそのまま物理に入れず、`servo_slew_deg_per_s` / `thrust_slew_per_s`
で平滑化してから使い、**平滑化後の値を観測にも返している**。つまりポリシーは
「レート制限を含むプラント」を制御対象として学習しており、制限はローパスとして
ポリシーの高周波成分を吸収していた。

実機側にこれが無いと、ポリシーが出す「毎ステップ符号が反転する飽和指令」がそのまま
サーボへ行く。実測 (2026-08-21):

* 制限あり (250 deg/s): 実サーボ角は 0..30 deg に収まり、**符号反転 0 回/秒**
* 制限なし:             実サーボ角が ±90 deg を往復し、**符号反転 49.7 回/秒**

学習時の観測 (`servo_n`) の std は 0.192 = 実角度 ±17 deg 相当で、制限ありのほうと
一致する。制限なしだと観測自体が学習分布から外れ、振動が持続する正帰還になる。
"""

from __future__ import annotations

import numpy as np


def slew(current, target, max_rate: float, dt: float):
    """`current` を `target` へ、1 ステップあたり最大 `max_rate * dt` だけ近づける。

    `max_rate <= 0` なら制限しない（`target` をそのまま返す）。
    単位は current / target / max_rate で揃っていればなんでもよい。
    """
    target = np.asarray(target, dtype=float)
    if max_rate <= 0.0:
        return target
    step = max_rate * dt
    return current + np.clip(target - current, -step, step)
