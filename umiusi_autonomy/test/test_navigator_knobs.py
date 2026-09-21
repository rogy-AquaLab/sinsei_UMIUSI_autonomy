"""navigator の現場つまみの回帰テスト。

`direct` モードでは navigator が**自分でスラスタを叩く**ので、cap と符号は
`classical_attitude` と同じ意味でなければならない。片方だけ違うと、
「どちらのノードで動かしているか」で復旧手段が変わる (known_issues B-14 / B-17)。
"""
import importlib.util
from pathlib import Path

import pytest

NAV = (Path(__file__).resolve().parents[2]
       / "umiusi_autonomy" / "umiusi_autonomy" / "navigator_node.py")


@pytest.fixture(scope="module")
def mod():
    spec = importlib.util.spec_from_file_location("navigator_node", NAV)
    m = importlib.util.module_from_spec(spec)
    try:
        spec.loader.exec_module(m)
    except ImportError as e:
        pytest.skip(f"ROS の依存が無い: {e}")
    return m


class Fake:
    """`_command_thrusters` を本物のまま呼ぶ器。publish 先だけ差し替える。"""

    def __init__(self, mod, cap):
        self._max_duty, self._publish = cap, True
        self._thrust_sign = [1.0] * 4
        self._servo_sign = [1.0] * 4
        self._servo_range_deg = 90.0
        self.sent = []
        self._pubs = {p: type("P", (), {"publish": lambda _s, msg, q=self: q.sent.append(msg)})()
                      for p in mod.POSITIONS}
        self._command_thrusters = mod.NavigatorNode._command_thrusters.__get__(self)


@pytest.mark.parametrize("cap,want", [(0.0, 0.0), (0.25, 0.25), (0.4, 0.4), (1.0, 0.8)])
def test_max_dutyが上限として効く(mod, cap, want):
    """**`max_duty=0` は「上限なし」ではなく「出力を止める」。**

    現場で推力を切りたい操縦者が打つのは 0 で、そこで生の duty が素通しになるのは
    危険側に外れる。`classical_attitude` の `_emit` は 0 で 0 に clip する。"""
    f = Fake(mod, cap)
    f._command_thrusters([0.0] * 4 + [0.8] * 4)
    got = max(abs(x.duty_cycle) for x in f.sent)
    assert got == pytest.approx(want), f"max_duty={cap} で duty {got}"


def test_符号は出口だけで掛かる(mod):
    f = Fake(mod, 1.0)
    f._thrust_sign = [1.0, -1.0, 1.0, -1.0]
    f._command_thrusters([0.0] * 4 + [0.5] * 4)
    assert [x.duty_cycle for x in f.sent] == pytest.approx([0.5, -0.5, 0.5, -0.5])
