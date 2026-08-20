"""ImuSanity の単体テスト。実機 (alexandrite / BNO055) で実際に観測した化け値を使う。"""

import math

import pytest

from umiusi_rl_control.imu_sanity import GYRO_FULL_SCALE, ImuSanity


def q_identity():
    return (1.0, 0.0, 0.0, 0.0)


def q_roll(deg):
    h = math.radians(deg) / 2.0
    return (math.cos(h), math.sin(h), 0.0, 0.0)


def test_正常なサンプルは通る():
    s = ImuSanity()
    out, reason = s.update(q_identity(), (0.001, 0.002, -0.001))
    assert reason is None
    assert out is not None
    assert s.accepted == 1 and s.rejected == 0


def test_ゼロクォータニオンを弾く():
    """実機の静止 60 秒で 2 件観測した |q| ~= 0 のサンプル。"""
    s = ImuSanity()
    s.update(q_identity(), (0.0, 0.0, 0.0))
    out, reason = s.update((0.0001, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    assert reason is not None and "ノルム" in reason
    assert out.quat == pytest.approx(q_identity())      # 直前の有効値を保持


def test_ジャイロのフルスケール化けを弾く():
    """3 軸とも ±35.6 rad/s に張り付く読み出し化け (int16 フルスケール)。"""
    s = ImuSanity()
    s.update(q_identity(), (0.0, 0.0, 0.0))
    out, reason = s.update(q_identity(), (-35.3658, 35.6134, -35.6549))
    assert reason is not None and "角速度" in reason
    assert "フルスケール" in reason                       # 化けだと明示されること
    assert out.gyro == (0.0, 0.0, 0.0)


def test_姿勢の急変を弾く():
    """0.5 秒で -3deg -> -170deg -> -4deg という実測の跳躍。"""
    s = ImuSanity(max_step_deg=30.0)
    s.update(q_roll(-3.0), (0.0, 0.0, 0.0))
    out, reason = s.update(q_roll(-170.0), (0.1, 0.1, 0.1))
    assert reason is not None and "急変" in reason
    assert out.quat == pytest.approx(q_roll(-3.0))


def test_実際の運動は通す():
    """50 Hz で 1 サンプル 10 度 (= 500 deg/s) までは正常な運動として通す。"""
    s = ImuSanity(max_step_deg=30.0)
    s.update(q_roll(0.0), (0.0, 0.0, 0.0))
    for deg in (10.0, 20.0, 30.0, 40.0):
        out, reason = s.update(q_roll(deg), (0.5, 0.0, 0.0))
        assert reason is None, f"{deg} deg で誤って弾いた: {reason}"


def test_符号反転したクォータニオンを急変と誤判定しない():
    """q と -q は同じ姿勢。符号が反転しただけで弾いてはいけない。"""
    s = ImuSanity()
    s.update(q_roll(10.0), (0.0, 0.0, 0.0))
    w, x, y, z = q_roll(10.0)
    out, reason = s.update((-w, -x, -y, -z), (0.0, 0.0, 0.0))
    assert reason is None


def test_NaNとInfを弾く():
    s = ImuSanity()
    s.update(q_identity(), (0.0, 0.0, 0.0))
    for bad in (float("nan"), float("inf")):
        _, reason = s.update(q_identity(), (bad, 0.0, 0.0))
        assert reason is not None and "NaN/Inf" in reason


def test_連続棄却でstaleになる():
    s = ImuSanity(stale_after=3)
    s.update(q_identity(), (0.0, 0.0, 0.0))
    assert not s.stale
    for _ in range(4):
        s.update(q_identity(), (100.0, 0.0, 0.0))
    assert s.stale, "連続して弾いたら stale を立てること"
    s.update(q_identity(), (0.0, 0.0, 0.0))
    assert not s.stale, "有効値が来たら回復すること"


def test_最初のサンプルが化けていても落ちない():
    s = ImuSanity()
    out, reason = s.update((0.0, 0.0, 0.0, 0.0), (0.0, 0.0, 0.0))
    assert reason is not None
    assert out is None                                   # 保持すべき値がまだ無い


def test_フルスケール定数がBNO055の仕様と一致する():
    # int16 の最大値 / 16 LSB per deg/s -> rad/s
    assert GYRO_FULL_SCALE == pytest.approx(math.radians(32767 / 16.0), abs=0.01)
