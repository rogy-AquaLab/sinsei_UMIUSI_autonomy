"""断の検出 (IMU / 速度指令) の回帰テスト。

どちらも「黙って走り続ける」のを防ぐためのもの。実機で起きたのは速度指令の側で、
2026-09-13 の bag に `manual_target_generator: Target is not updated` が 421 件残っている。
当時の姿勢制御器は最後の指令のまま走り続ける実装だった。

ノードを立てずに、時計と受信時刻だけを差し替えて判定ロジックを突く。
"""
import importlib.util
from pathlib import Path

import pytest

NODE = (Path(__file__).resolve().parents[2]
        / "umiusi_autonomy" / "umiusi_autonomy" / "classical_attitude_node.py")


@pytest.fixture(scope="module")
def mod():
    spec = importlib.util.spec_from_file_location("classical_attitude_node", NODE)
    m = importlib.util.module_from_spec(spec)
    try:
        spec.loader.exec_module(m)
    except ImportError as e:
        pytest.skip(f"ROS の依存が無い: {e}")
    return m


class Fake:
    """判定に要る属性だけ持つ器。`_imu_stale` / `_vel_stale` を素の関数として呼ぶ。"""

    def __init__(self, mod, now=100.0, imu_timeout=1.0, vel_timeout=1.0):
        self._t = now
        self._imu_timeout, self._vel_timeout = imu_timeout, vel_timeout
        self._imu_t = self._vel_t = None
        self._imu_stale = mod.ClassicalAttitudeNode._imu_stale.__get__(self)
        self._vel_stale = mod.ClassicalAttitudeNode._vel_stale.__get__(self)

    def _now(self):
        return self._t


def test_IMUが来ていなければ断とみなす(mod):
    """**一度も来ていない**のと「来ていたが途切れた」を同じに扱う。

    起動直後に arm されると、姿勢の基準が無いまま推力を出すことになる。"""
    f = Fake(mod)
    assert f._imu_stale()


def test_IMUが来ていれば断ではない(mod):
    f = Fake(mod)
    f._imu_t = f._t - 0.02
    assert not f._imu_stale()


def test_IMUが途切れたら断になる(mod):
    f = Fake(mod)
    f._imu_t = f._t - 1.5
    assert f._imu_stale()


def test_IMUの断はtimeout0で無効にできる(mod):
    f = Fake(mod, imu_timeout=0.0)
    assert not f._imu_stale()          # 一度も来ていなくても止めない


def test_速度指令のデッドマンは一度来るまで武装しない(mod):
    """指令を出す相手が居ない構成 (姿勢保持だけ) で誤検出しないこと。"""
    f = Fake(mod)
    assert not f._vel_stale()


def test_速度指令が途切れたら効く(mod):
    f = Fake(mod)
    f._vel_t = f._t - 1.5
    assert f._vel_stale()


def test_速度指令が来ていれば効かない(mod):
    f = Fake(mod)
    f._vel_t = f._t - 0.1
    assert not f._vel_stale()


def test_デッドマンはtimeout0で無効にできる(mod):
    f = Fake(mod, vel_timeout=0.0)
    f._vel_t = f._t - 100.0
    assert not f._vel_stale()


def test_既定はどちらも有効(mod):
    """**既定で切れていたら意味が無い。** rl_attitude_node の vel_timeout は既定 0 だが、
    こちらはスラスタを叩く唯一のノードなので既定で入れる。"""
    src = NODE.read_text()
    for name in ("imu_timeout", "vel_timeout"):
        i = src.index(f'declare_parameter("{name}"')
        assert "0.0" not in src[i:i + 60], f"{name} の既定が 0 (無効) になっている"


# --- 制御の計算が例外を投げたとき --------------------------------------------------


class FakeTick:
    """`_tick` の骨だけ回す。rclpy を立てずに例外時の分岐を突く。"""

    def __init__(self, mod, fail_forever=True):
        self.mod, self.boom, self.logs = mod, fail_forever, []
        self._fail, self.emitted, self.disarmed = 0, 0, False
        self._arm = type("A", (), {"armed": True, "disarm": self._disarm})()
        self._tick = mod.ClassicalAttitudeNode._tick.__get__(self)

    def _disarm(self, reason=""):
        self.disarmed, self._arm.armed = True, False

    def _tick_impl(self):
        if not self._arm.armed:
            return
        if self.boom:
            raise ValueError("わざと壊した")

    def _emit(self, _action):
        self.emitted += 1

    def get_logger(self):
        rec = self.logs.append
        return type("L", (), {"error": lambda _s, m, **k: rec(m),
                              "warning": lambda _s, m, **k: rec(m)})()


def test_一周期の失敗ではrunを止めない(mod):
    """1 サンプルの化けで本番の run を終わらせない。0 を出して次の周期へ行く。"""
    f = FakeTick(mod)
    f._tick()
    assert f.emitted == 1 and not f.disarmed and f._fail == 1


def test_失敗が続いたらdisarmする(mod):
    f = FakeTick(mod)
    for _ in range(mod.FAIL_LIMIT):
        f._tick()
    assert f.disarmed and f._fail == mod.FAIL_LIMIT


def test_disarm中は復帰したと言わない(mod):
    """制御の経路を通らずに抜けているだけなので、直った証拠が無い。"""
    f = FakeTick(mod)
    for _ in range(mod.FAIL_LIMIT):
        f._tick()
    n = len(f.logs)
    f._tick()                      # disarm 済み -> _tick_impl は即 return する
    assert not any("復帰" in m for m in f.logs[n:]), f.logs[n:]
    assert f._fail == mod.FAIL_LIMIT


def test_直れば復帰する(mod):
    f = FakeTick(mod)
    f._tick()
    f.boom = False
    f._tick()
    assert f._fail == 0 and any("復帰" in m for m in f.logs)


# --- disarm が保護を外したまま古い指令を残さないこと ------------------------------


class FakeDetach:
    """`_detach_all` を本物のまま呼ぶための器 (publish はしない)。"""

    def __init__(self, mod, vel_cmd_param=0.0):
        import numpy as np
        self._ctl = type("C", (), {"reset": lambda _s: None})()
        self._alloc = type("A", (), {"reset": lambda _s: None})()
        self._publish = False
        self._param = vel_cmd_param
        self._vel_cmd = np.array([0.9, 0.0, 0.0])     # テレオペで入った古い速度
        self._vel_t, self._vel_dead, self._fail = 123.0, True, mod.FAIL_LIMIT
        self._yaw_sp, self._yaw_t_prev = 1.0, 5.0
        self._servo_cmd = np.ones(4)
        self._duty_cmd = np.ones(4)
        self._action = np.ones(8)
        self._detach_all = mod.ClassicalAttitudeNode._detach_all.__get__(self)

    def get_parameter(self, name):
        assert name == "vel_cmd"
        return type("P", (), {"value": self._param})()


def test_disarmで古い速度指令が残らない(mod):
    """**「指令が途切れた → disarm → 再 arm」で古い速度のまま走り出さない。**

    時計 (`_vel_t`) だけ戻して指令を残すと、デッドマンは武装しないのに速度は残る、
    という一番悪い組み合わせになる。"""
    f = FakeDetach(mod)
    f._detach_all()
    assert f._vel_cmd[0] == 0.0 and f._vel_t is None and f._vel_dead is False


def test_disarmでvel_cmdはparamの値に戻る(mod):
    """巡航を `vel_cmd` で入れて使う構成を、disarm が壊さないこと。"""
    f = FakeDetach(mod, vel_cmd_param=0.4)
    f._detach_all()
    assert f._vel_cmd[0] == pytest.approx(0.4)


def test_disarmで失敗カウンタが戻る(mod):
    """戻さないと、再 arm 後の 1 回目の失敗で即 disarm になる。"""
    f = FakeDetach(mod)
    f._detach_all()
    assert f._fail == 0
