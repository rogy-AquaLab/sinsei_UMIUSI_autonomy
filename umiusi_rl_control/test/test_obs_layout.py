"""観測レイアウトの単体テスト。

観測の並びは **sim との契約** で、ずれると方策が黙って別の入力を読む (golden 検証で
起動時には落ちるが、ここで早く気付けるようにする)。特に `max_duty` は末尾固定 —
sim 側の warm start (初層のゼロパディングで 17 次元の重みを引き継ぐ) がこの位置を
前提にしている。
"""
import numpy as np
import pytest

rclpy = pytest.importorskip("rclpy")

from umiusi_rl_control import rl_attitude_node as N  # noqa: E402


class _Stub:
    """`_build_obs` が触る属性だけを持つスタブ (Node を立てずにレイアウトだけ見る)。"""

    _build_obs = N.RlAttitudeNode._build_obs

    def __init__(self, max_duty=0.25, hold_yaw=True):
        self._imu = type("Imu", (), {"quat": (1.0, 0.0, 0.0, 0.0),
                                     "gyro": (0.1, 0.2, 0.3)})()
        self._target_quat = np.array([1.0, 0.0, 0.0, 0.0])
        self._hold_yaw = hold_yaw
        self._prev_action = np.arange(N.ACT_DIM, dtype=float) / 10.0
        self._max_duty = max_duty


V_CMD = np.array([0.4, 0.0, 0.0])


@pytest.mark.parametrize("obs_dim", N.OBS_DIMS_SUPPORTED)
def test_観測は宣言した次元ちょうどになる(obs_dim):
    assert _Stub()._build_obs(V_CMD, obs_dim).shape == (obs_dim,)


def test_max_dutyは末尾に生値で入る():
    obs = _Stub(max_duty=0.3)._build_obs(V_CMD, N.OBS_DIM_CAP)
    assert obs[-1] == pytest.approx(0.3)          # 正規化しない (obs_norm.npz 側で素通し)
    # 末尾以外は 17 次元のときと完全に一致する = warm start の前提
    assert obs[:-1] == pytest.approx(_Stub()._build_obs(V_CMD, N.OBS_DIM))


def test_18次元と17次元でprev_actionの位置がずれない():
    # 深度スーパーバイザは 17 と 18 のモデル間で prev_action を共有する
    o17 = _Stub()._build_obs(V_CMD, N.OBS_DIM)
    o18 = _Stub()._build_obs(V_CMD, N.OBS_DIM_CAP)
    start = 3 + 3 + 3
    assert o17[start:start + N.ACT_DIM] == pytest.approx(o18[start:start + N.ACT_DIM])


def test_速度指令を持つのは17と18だけ():
    assert set(N.OBS_DIMS_WITH_VEL) == {N.OBS_DIM, N.OBS_DIM_CAP}
    o14 = _Stub()._build_obs(V_CMD, N.OBS_DIM_NO_VEL)
    assert o14.shape == (N.OBS_DIM_NO_VEL,)
    # v_cmd が入っていないので gyro の直後は prev_action
    assert o14[6:] == pytest.approx(_Stub()._prev_action)


def test_未対応の次元は黙って通さない():
    with pytest.raises(ValueError, match="次元"):
        _Stub()._build_obs(V_CMD, 25)
