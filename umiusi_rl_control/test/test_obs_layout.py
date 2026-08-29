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


# **姿勢は identity にしない。** 目標と現在姿勢を両方 identity にすると `ori_err` が恒等的に
# ゼロになり、観測の先頭 3 次元が一度も検証されない。レビューで mutation を注入して実証された:
# `ori_err` を zeros に置換 / `YAW_IDX` を間違える / `mju_sub_quat` の符号を反転 のどれでも
# 全テストが緑のまま通っていた。**golden が見ない穴を埋めるのがこのテストの主旨**なので、
# 一番安全側に効く次元 (姿勢誤差の符号) が素通りするのでは意味が無い。
_CUR_QUAT = (0.9689, 0.1435, 0.0958, 0.1794)      # roll/pitch/yaw いずれも非ゼロ
_TARGET_QUAT = np.array([0.9950, 0.0, 0.0, 0.0998])   # yaw だけずれた目標
# **契約を直接書く。** `N.YAW_IDX` を参照すると、定数を間違える mutation でテストの期待値も
# 一緒に動いてしまい検出できない (実際に mutation で確認した)。REP-103 body frame は
# x-fwd / y-left / z-up なので、回転ベクトルの yaw 成分は index 2。
_YAW_IDX = 2


class _Stub:
    """`_build_obs` が触る属性だけを持つスタブ (Node を立てずにレイアウトだけ見る)。"""

    _build_obs = N.RlAttitudeNode._build_obs
    # 観測に入る duty 上限は**ミキサに渡す値と同じもの**でなければならない (レンチモードの
    # 「モード 1.0 = その上限での全権限」という約束)。実装は 1 箇所に寄せてあるので、
    # スタブもその 1 箇所を借りる。
    _obs_max_duty = N.RlAttitudeNode._obs_max_duty

    def __init__(self, max_duty=0.25, hold_yaw=True):
        self._imu = type("Imu", (), {"quat": _CUR_QUAT, "gyro": (0.1, 0.2, 0.3)})()
        self._target_quat = _TARGET_QUAT
        self._hold_yaw = hold_yaw
        self._prev_action = np.arange(N.ACT_DIM, dtype=float) / 10.0
        self._max_duty = max_duty


def _expected_ori_err(hold_yaw=True):
    """実装と独立に期待値を作る (`mju_sub_quat` は MuJoCo と bit 一致が検証済みの実装)。"""
    err = N.mju_sub_quat(_TARGET_QUAT, np.array(_CUR_QUAT, dtype=float))
    if not hold_yaw:
        err[_YAW_IDX] = 0.0
    return err


V_CMD = np.array([0.4, 0.0, 0.0])


@pytest.mark.parametrize("obs_dim", N.OBS_DIMS_SUPPORTED)
def test_観測は宣言した次元ちょうどになる(obs_dim):
    assert _Stub()._build_obs(V_CMD, obs_dim).shape == (obs_dim,)


def test_ori_errが先頭3次元に入る():
    """レビューで指摘された穴。ここが無いと姿勢誤差の符号を壊しても誰も気付けない。"""
    want = _expected_ori_err()
    assert np.any(np.abs(want) > 1e-3), "テストデータが縮退している (ori_err がほぼゼロ)"
    for obs_dim in N.OBS_DIMS_SUPPORTED:
        obs = _Stub()._build_obs(V_CMD, obs_dim)
        assert obs[:3] == pytest.approx(want), obs_dim


def test_YAW_IDXがREP103のz軸を指している():
    """回転ベクトルの yaw 成分は REP-103 (x-fwd / y-left / z-up) では index 2。
    ここを間違えると `hold_yaw=false` が別の軸を潰す。"""
    assert N.YAW_IDX == _YAW_IDX


def test_hold_yawがfalseならyaw成分だけ落ちる():
    on = _Stub(hold_yaw=True)._build_obs(V_CMD, N.OBS_DIM_CAP)
    off = _Stub(hold_yaw=False)._build_obs(V_CMD, N.OBS_DIM_CAP)
    assert off[_YAW_IDX] == pytest.approx(0.0)
    assert on[_YAW_IDX] != pytest.approx(0.0)      # 落とす前は非ゼロ = テストが効いている
    keep = [i for i in range(3) if i != _YAW_IDX]
    assert off[keep] == pytest.approx(on[keep])     # roll/pitch は変わらない
    assert off[3:] == pytest.approx(on[3:])         # それ以降も変わらない


def test_max_dutyは末尾に入る():
    obs = _Stub(max_duty=0.3)._build_obs(V_CMD, N.OBS_DIM_CAP)
    assert obs[-1] == pytest.approx(0.3)          # 正規化しない (obs_norm.npz 側で素通し)
    # 末尾以外は 17 次元のときと完全に一致する = sim 側 warm start (初層ゼロパディング) の前提
    assert obs[:-1] == pytest.approx(_Stub()._build_obs(V_CMD, N.OBS_DIM))


@pytest.mark.parametrize(("setting", "want"), [(0.1, 0.2), (0.25, 0.25), (0.5, 0.4)])
def test_観測のmax_dutyは学習分布にクランプされる(setting, want):
    """クリップの実値はオペレータ設定のまま、**観測に入る値だけ**を学習分布へ丸める。
    範囲外をそのまま入れると、warm start でゼロ padding された新次元へ学習時に一度も
    見ていない値が入る。"""
    stub = _Stub(max_duty=setting)
    assert stub._build_obs(V_CMD, N.OBS_DIM_CAP)[-1] == pytest.approx(want)
    assert stub._max_duty == setting               # 実際のクリップ値は変えない


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
    want = {"ori_err": _expected_ori_err(), "gyro": np.array(stub._imu.gyro), "v_cmd": V_CMD,
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


@pytest.mark.parametrize("obs_dim", [N.OBS_DIM, N.OBS_DIM_NO_VEL])
def test_既存次元はobs_fieldsが無くても警告だけで通す(obs_dim):
    c = _Checker()
    c._check_obs_fields(_Runner(obs_dim), "export")
    assert len(c.warnings) == 1


@pytest.mark.parametrize("fields", [None, []])
def test_18次元はobs_fieldsが必須(fields):
    """後方互換で警告だけにしてよいのは「既に出回っていて直せない」バンドルの話。
    18 次元はこの機能と同時に生まれたので守るべき既存が無く、しかも並びを取り違えて
    一番困るのが末尾に max_duty を足したこの次元。"""
    with pytest.raises(ValueError, match="必須"):
        _Checker()._check_obs_fields(_Runner(N.OBS_DIM_CAP, fields), "export")


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
