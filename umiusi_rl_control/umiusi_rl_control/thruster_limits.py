"""スラスタ指令のレート制限。

  * 値は sim と揃えること (configs/umiusi.yaml の servo_slew_deg_per_s / thrust_slew_per_s)
  * 外さないこと — 方策はこの制限を含むプラントを対象に学習している
  * 経緯と実測: known_issues A-11
"""

from __future__ import annotations

import numpy as np


def slew(current, target, max_rate: float, dt: float):
    """`current` を `target` へ、1 ステップあたり最大 `max_rate * dt` だけ近づける。

    `max_rate <= 0` で無制限。単位は 3 つで揃っていればよい。
    """
    target = np.asarray(target, dtype=float)
    if max_rate <= 0.0:
        return target
    step = max_rate * dt
    return current + np.clip(target - current, -step, step)
