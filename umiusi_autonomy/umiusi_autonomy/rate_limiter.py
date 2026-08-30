"""目標レートに追従するフレーム間引き。ROS 非依存なので単体でテストできる。

素朴に「前回処理した時刻から一定時間空いたら通す」方式にすると、入力周期が
上限周期より少しでも短いときに必ず 1 フレームおきに落ちて、目標の半分近くまで下がる。

    上限 10 Hz (周期 100 ms) に 15 Hz (周期 66.7 ms) を入れた場合:
      t=0    通す        (次は 100 ms 以降)
      t=66.7 落とす      (100 ms に満たない)
      t=133  通す        (次は 233 ms 以降)   <- 実質 2 フレームに 1 回
      ...  => 7.5 Hz     実機でも 7.9 Hz しか出ていなかった

期限を「実際に通した時刻」ではなく「前回の期限」から進めれば、位相が入力に追従して
目標レートに最も近いフレームを選べるようになる。処理が遅れて期限を大きく過ぎた場合は
まとめて追いつこうとせず、その時点から張り直す (バースト防止)。
"""

from __future__ import annotations


class RateLimiter:
    """目標レート以下になるようフレームを間引く。

    Parameters
    ----------
    rate_hz:
        目標レート。0 以下なら制限しない (すべて通す)。
    """

    def __init__(self, rate_hz: float) -> None:
        self.rate_hz = float(rate_hz)
        self.period = (1.0 / rate_hz) if rate_hz > 0 else 0.0
        self._deadline: float | None = None
        self.passed = 0
        self.dropped = 0

    @property
    def enabled(self) -> bool:
        return self.period > 0.0

    def allow(self, stamp: float) -> bool:
        """時刻 stamp [s] に届いたフレームを処理してよいか。"""
        if not self.enabled:
            self.passed += 1
            return True

        if self._deadline is None:          # 最初の 1 枚は必ず通す
            self._deadline = stamp + self.period
            self.passed += 1
            return True

        if stamp < self._deadline:
            self.dropped += 1
            return False

        # 期限を 1 周期ぶん進める。位相を入力に引きずられないようにするのが要点。
        self._deadline += self.period
        # 大きく遅れていたら (処理が詰まった等) その時点から張り直す。
        # ここで過去の期限を積み残すと、詰まりが解けた瞬間に連続で通してしまう。
        if self._deadline <= stamp:
            self._deadline = stamp + self.period
        self.passed += 1
        return True

    def reset(self) -> None:
        self._deadline = None
