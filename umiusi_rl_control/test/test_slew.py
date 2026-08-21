"""指令のレート制限。sim (umiusi_sim/physics/thruster.py) と同じ挙動であること。

実機側にこれが無かったため、ポリシーが出す「毎ステップ符号が反転する飽和指令」が
そのままサーボへ行き、追従できずに震えるだけになっていた (2026-08-21 の実機、
実測 12.5 Hz で ±90° 往復・平均推力ほぼ 0)。sim は同じ指令をレート制限で平滑化して
から物理に入れており、観測にもその平滑化後の値が返っていた。
"""

import numpy as np
import pytest

from umiusi_rl_control.thruster_limits import slew


def test_1ステップの変化がmax_rate_dtに制限される():
    out = slew(np.zeros(4), np.full(4, 90.0), 250.0, 0.02)   # 250 deg/s * 20 ms = 5 deg
    assert out == pytest.approx(np.full(4, 5.0))


def test_目標が近ければそのまま到達する():
    out = slew(np.zeros(4), np.full(4, 1.0), 250.0, 0.02)
    assert out == pytest.approx(np.full(4, 1.0))


def test_負の方向にも同じだけ制限される():
    out = slew(np.zeros(4), np.full(4, -90.0), 250.0, 0.02)
    assert out == pytest.approx(np.full(4, -5.0))


def test_max_rate_が_0_以下なら制限しない():
    for rate in (0.0, -1.0):
        out = slew(np.zeros(4), np.full(4, 90.0), rate, 0.02)
        assert out == pytest.approx(np.full(4, 90.0))


def test_飽和した往復指令が中立付近に収まる():
    """±90 を毎ステップ交互に指令しても、実際の指令は ±5 度の範囲で震えるだけ。

    これが sim で起きていたこと。実機でも同じにするのがこの制限の目的。
    """
    cur = np.zeros(4)
    seen = []
    for i in range(200):
        target = np.full(4, 90.0 if i % 2 == 0 else -90.0)
        cur = slew(cur, target, 250.0, 0.02)
        seen.append(cur.copy())
    seen = np.array(seen)
    assert np.max(np.abs(seen)) <= 5.0 + 1e-9, "レート制限を超えて動いている"


def test_一方向の指令なら追従していく():
    cur = np.zeros(4)
    for _ in range(20):                       # 20 ステップ * 5 deg = 100 > 90
        cur = slew(cur, np.full(4, 90.0), 250.0, 0.02)
    assert cur == pytest.approx(np.full(4, 90.0))
