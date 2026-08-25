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


# --- OBS_FIELDS (meta.json との突き合わせ表) が実際の組み立てと一致していること ---
# golden 検証は記録済みの観測をそのままネットに流すだけなので、**組み立て順の取り違えを
# 検出できない**。その穴を埋めるのが OBS_FIELDS なので、表そのものがずれていたら無意味。

@pytest.mark.parametrize("obs_dim", N.OBS_DIMS_SUPPORTED)
def test_OBS_FIELDSの幅の合計が次元と一致する(obs_dim):
    assert sum(w for _, w in N.OBS_FIELDS[obs_dim]) == obs_dim


@pytest.mark.parametrize("obs_dim", N.OBS_DIMS_SUPPORTED)
def test_OBS_FIELDSの並びが実際の組み立てと一致する(obs_dim):
    """各フィールドの区間に、実際にその値が入っていることを確かめる。"""
    stub = _Stub(max_duty=0.3)
    obs = stub._build_obs(V_CMD, obs_dim)
    want = {"gyro": np.array(stub._imu.gyro), "v_cmd": V_CMD,
            "prev_action": stub._prev_action, "max_duty": np.array([0.3])}
    i = 0
    for name, width in N.OBS_FIELDS[obs_dim]:
        if name in want:
            assert obs[i:i + width] == pytest.approx(want[name]), (obs_dim, name)
        i += width
    assert i == obs_dim


# --- meta.json の obs_fields 照合 (Node を立てずにメソッドだけ呼ぶ) ---

class _Runner:
    def __init__(self, obs_dim, fields=None):
        self.obs_dim = obs_dim
        self.meta = {} if fields is None else {"obs_fields": fields}


class _Checker:
    """`_check_obs_fields` はロガーしか使わないので、それだけ差し替える。"""

    _check_obs_fields = N.RlAttitudeNode._check_obs_fields

    def __init__(self):
        self.warnings, self.infos = [], []
        me = self

        class _Log:
            def warning(self, m, **kw):
                me.warnings.append(m)

            def info(self, m, **kw):
                me.infos.append(m)

        self._log = _Log()

    def get_logger(self):
        return self._log


def _fields(obs_dim):
    return [[n, w] for n, w in N.OBS_FIELDS[obs_dim]]


@pytest.mark.parametrize("obs_dim", N.OBS_DIMS_SUPPORTED)
def test_一致するobs_fieldsは通る(obs_dim):
    c = _Checker()
    c._check_obs_fields(_Runner(obs_dim, _fields(obs_dim)), "export")
    assert c.infos and not c.warnings


def test_obs_fieldsが無ければ警告だけで通す():
    c = _Checker()
    c._check_obs_fields(_Runner(N.OBS_DIM), "export")     # 既存の 17 次元バンドル
    assert len(c.warnings) == 1


def test_並びが違えば起動しない():
    swapped = _fields(N.OBS_DIM_CAP)
    swapped[0], swapped[1] = swapped[1], swapped[0]        # ori_err と gyro を入れ替え
    with pytest.raises(ValueError, match="観測レイアウト"):
        _Checker()._check_obs_fields(_Runner(N.OBS_DIM_CAP, swapped), "export")


def test_max_dutyを先頭に置いたバンドルは弾く():
    # このノードの実装ミスと対称な、sim 側が先頭に置いた場合
    bad = [["max_duty", 1]] + _fields(N.OBS_DIM)
    with pytest.raises(ValueError, match="観測レイアウト"):
        _Checker()._check_obs_fields(_Runner(N.OBS_DIM_CAP, bad), "export")


def test_幅の合計が次元と合わなければ弾く():
    bad = _fields(N.OBS_DIM_CAP)[:-1]                      # max_duty を落とす = 17 分しかない
    with pytest.raises(ValueError, match="幅の合計"):
        _Checker()._check_obs_fields(_Runner(N.OBS_DIM_CAP, bad), "export")


@pytest.mark.parametrize("bad", [["ori_err"], [["ori_err"]], [["ori_err", "three"]], "ori_err"])
def test_形式が壊れていれば明示的に落ちる(bad):
    with pytest.raises(ValueError, match="obs_fields"):
        _Checker()._check_obs_fields(_Runner(N.OBS_DIM_CAP, bad), "export")
