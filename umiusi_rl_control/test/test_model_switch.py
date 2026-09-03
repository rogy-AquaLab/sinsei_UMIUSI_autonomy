"""深度モードの切替でモードの積分器がどう扱われるかの単体テスト。

待機していたモデルは「最後に使ったときのモードベクトル」を抱えたままになる。 再選択の
瞬間にその力がいきなり出ると危険で、sim には対応する状況が無い (env は方策 1 個・積分器 1 個)。
_select_model がそこを塞いでいる。

現状は水平・鉛直どちらのバンドルも direct 出力なのでこの経路は不活性だが、vert に
modes 系を載せた瞬間に効く。そのとき黙って壊れないようにするためのテスト。
"""
import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")

from umiusi_rl_control import rl_attitude_node as N  # noqa: E402
from umiusi_rl_control.mode_action import MODE_DIM, ModeAction  # noqa: E402

POSITIONS = ("lf", "lb", "rb", "rf")
CONTRACT = {
    "mode_names": ["fx", "fy", "fz", "tx", "ty", "tz"],
    "mode_sign_columns": ["fx", "fy", "tz", "fz", "tx", "ty"],
    "mode_signs": {"lf": [1, -1, -1, 1, 1, -1], "lb": [1, 1, -1, 1, 1, 1],
                   "rb": [-1, 1, -1, 1, -1, 1], "rf": [-1, -1, -1, 1, -1, -1]},
    "mode_slew_per_s": 2.0, "deadband_frac": 0.02, "thrust_per_cmd": 30.0,
    "thrust_curve_exp": 2.0, "servo_range_deg": 90.0, "control_rate_hz": 50.0,
}


class _Model:
    """_select_model が触る属性だけを持つ偽モデル。"""

    def __init__(self, modes=False):
        self.mode_action = ModeAction(CONTRACT, POSITIONS) if modes else None


class _Stub:
    """_select_model / _reset_mode_state が触る状態だけを持つスタブ (Node を立てない)。"""

    _select_model = N.RlAttitudeNode._select_model
    _reset_mode_state = N.RlAttitudeNode._reset_mode_state

    def __init__(self, model=None, vert_model=None):
        self._active_model = None
        self._model = model
        self._vert_model = vert_model
        self._prev_action = np.ones(N.ACT_DIM)


def _wind_up(model, steps=25):
    """積分器を飽和側へ振っておく (= 待機中に抱えている状態を作る)。"""
    for _ in range(steps):
        model.mode_action.step(np.ones(MODE_DIM), 0.25, 1.0 / 50.0)
    assert np.any(model.mode_action.modes != 0.0), "テストの前提が崩れている"


def test_初回選択でも積分器はゼロから始まる():
    s, m = _Stub(), _Model(modes=True)
    _wind_up(m)
    s._select_model(m)
    assert np.all(m.mode_action.modes == 0.0)


def test_切替で新しく選ばれたほうの積分器がゼロに戻る():
    """これが本題。 待機していた側を再選択した瞬間に古い力が出ないこと。"""
    s, horiz, vert = _Stub(), _Model(modes=True), _Model(modes=True)
    s._select_model(horiz)
    _wind_up(vert)                      # vert は前に使ったときの状態を抱えている
    s._select_model(vert)               # 深度モードへ切替
    assert np.all(vert.mode_action.modes == 0.0), "待機明けの積分器が残っている"


def test_同じモデルが続く間は積分器を消さない():
    """毎 tick リセットしたら積分そのものが成立しない。"""
    s, m = _Stub(), _Model(modes=True)
    s._select_model(m)
    _wind_up(m)
    before = m.mode_action.modes.copy()
    s._select_model(m)                  # 次の tick、同じモデル
    assert np.array_equal(m.mode_action.modes, before), "同一モデルなのにリセットされた"


def test_直接出力のモデルでも落ちない():
    """mode_action が None のモデル (従来のバンドル) を挟んでも例外にならない。"""
    s, direct, modes = _Stub(), _Model(modes=False), _Model(modes=True)
    s._select_model(direct)
    _wind_up(modes)
    s._select_model(modes)
    assert np.all(modes.mode_action.modes == 0.0)
    s._select_model(direct)             # 逆向きの切替
    assert s._active_model is direct


# --- disarm で state が残らないこと (_detach_all -> _reset_mode_state) ------------------

def test_disarmで両方のモデルの積分器が消える():
    """arm したまま積んだ力が、再 armの最初の tick で出てはいけない。

    リセット漏れがあると、disarm 直前のモードベクトルぶんの力が次のarmでいきなり出る。
    _select_model は「切り替わったほう」しか消さないので、disarm 側で両方消す必要がある。
    """
    horiz, vert = _Model(modes=True), _Model(modes=True)
    s = _Stub(model=horiz, vert_model=vert)
    _wind_up(horiz)
    _wind_up(vert)
    s._reset_mode_state()
    assert np.all(horiz.mode_action.modes == 0.0), "水平側の積分器が残っている"
    assert np.all(vert.mode_action.modes == 0.0), "鉛直側の積分器が残っている"


def test_disarmで再arm時の切替判定がやり直される():
    """_active_model を残すと、再 armで同じモデルが選ばれたときリセットが飛ぶ。"""
    horiz = _Model(modes=True)
    s = _Stub(model=horiz)
    s._select_model(horiz)
    assert s._active_model is horiz
    s._reset_mode_state()
    assert s._active_model is None


def test_disarmでprev_actionもゼロに戻る():
    """観測の proprio は「自分が直前に出した指令」。指令を出していない間の値を残すと、
    再 armした最初の観測が実際とずれる。"""
    s = _Stub(model=_Model(modes=True))
    s._reset_mode_state()
    assert np.all(s._prev_action == 0.0)


def test_直接出力だけならprev_actionを触らない():
    """従来のバンドルの挙動を変えない (modes 系が 1 つも無ければ何もしない)。"""
    s = _Stub(model=_Model(modes=False), vert_model=None)
    s._reset_mode_state()
    assert np.all(s._prev_action == 1.0), "direct のみなのに prev_action が消された"
