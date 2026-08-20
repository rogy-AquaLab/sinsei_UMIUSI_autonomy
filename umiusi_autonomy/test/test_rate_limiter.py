"""RateLimiter の単体テスト。実機で観測したレートを使って回帰を防ぐ。"""

import pytest

from umiusi_autonomy.rate_limiter import RateLimiter


def measure(limiter, input_hz, seconds=10.0):
    """input_hz で届くフレームを流し、通ったレート [Hz] を返す。"""
    dt = 1.0 / input_hz
    n = int(seconds * input_hz)
    passed = sum(1 for i in range(n) if limiter.allow(i * dt))
    return passed / seconds


def old_style(input_hz, cap_hz, seconds=10.0):
    """修正前の実装 (通した時刻から一定時間空ける方式) を再現する。"""
    dt, period = 1.0 / input_hz, 1.0 / cap_hz
    last, passed = None, 0
    for i in range(int(seconds * input_hz)):
        t = i * dt
        if last is not None and (t - last) < period:
            continue
        last = t
        passed += 1
    return passed / seconds


@pytest.mark.parametrize("input_hz", [13.4, 15.0, 20.0, 30.0])
def test_目標レートに追従する(input_hz):
    """入力が上限より速いとき、出力は上限に十分近くなること。"""
    got = measure(RateLimiter(10.0), input_hz)
    assert got == pytest.approx(10.0, abs=0.6), f"{input_hz} Hz 入力で {got:.2f} Hz"


def test_修正前は目標の半分近くまで落ちていた():
    """回帰防止。実機では 15 Hz 入力 + 10 Hz 上限で 7.9 Hz しか出ていなかった。"""
    assert old_style(15.0, 10.0) == pytest.approx(7.5, abs=0.2)   # 欠陥の再現
    assert measure(RateLimiter(10.0), 15.0) > 9.4                  # 修正後


def test_実機で観測した組み合わせ():
    """13.4 Hz 入力 + 10 Hz 上限。実機の実測は 7.78 Hz だった。"""
    assert old_style(13.4, 10.0) < 8.0
    assert measure(RateLimiter(10.0), 13.4) > 9.0


def test_入力が上限より遅ければ素通し():
    for hz in (3.0, 7.0, 9.5):
        assert measure(RateLimiter(10.0), hz) == pytest.approx(hz, abs=0.2)


def test_上限0は無制限():
    lim = RateLimiter(0.0)
    assert not lim.enabled
    assert measure(lim, 30.0) == pytest.approx(30.0, abs=0.2)


def test_最初のフレームは必ず通す():
    assert RateLimiter(10.0).allow(123.456) is True


def test_詰まりが解けてもバーストしない():
    """長く止まったあと、溜まった期限をまとめて消化して連続通過させないこと。"""
    lim = RateLimiter(10.0)
    assert lim.allow(0.0)
    # 5 秒間フレームが来ない (処理が詰まった等)
    assert lim.allow(5.0)
    # 直後に高頻度で届いても、上限どおりに間引かれること。
    # 100 ms のあいだに 10 枚届いても、10 Hz 上限なら通るのは 1 枚まで。
    passed = sum(1 for i in range(1, 11) if lim.allow(5.0 + i * 0.01))   # 100 Hz で 10 枚
    assert passed <= 1, f"バーストして {passed} 枚通した"

    # さらに 1 秒ぶん 100 Hz で流しても、10 Hz を大きく超えないこと
    burst = sum(1 for i in range(1, 101) if lim.allow(5.1 + i * 0.01))
    assert burst <= 11, f"1 秒で {burst} 枚通した (上限 10 Hz のはず)"


def test_統計が取れる():
    lim = RateLimiter(10.0)
    for i in range(30):
        lim.allow(i / 30.0)
    assert lim.passed + lim.dropped == 30
    assert lim.passed == pytest.approx(10, abs=1)
