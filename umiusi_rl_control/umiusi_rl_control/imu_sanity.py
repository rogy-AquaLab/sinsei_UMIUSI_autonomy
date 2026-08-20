"""IMU の化けサンプルを弾く。ROS 非依存なので単体でテストできる。

実機 (BNO055) では物理的にありえないサンプルが混入することを確認している:

* ノルムが 0 のクォータニオン — 静止 60 秒で 2 件 (約 30 秒に 1 回)
* 角速度が 3 軸とも ±35.6 rad/s — BNO055 の int16 フルスケール
  (32767/16 = 2047.9 deg/s = 35.74 rad/s) と一致する読み出し化け
* 0.5 秒で -3° → -170° → -4° といった姿勢の跳躍 (運動中に出やすい)

`navigator_node` / `auto_target_generator` / `rl_attitude_node` はいずれも角速度を
ヨーレートとして、姿勢をそのまま制御に使うため、**1 発のスパイクで制御が跳ねる**。
受信側でここを通してから使う。

判定は「疑わしきは捨てる」。捨てたときは直前の有効値を返し、連続して捨て続けた場合は
`stale` が True になるので、呼び出し側でフェイルセーフに落とせる。

ただし **姿勢の跳躍だけは「捨て続ける」ことができない**。実機で、BNO055 の姿勢基準が
一度だけ 169° 飛び、飛んだ先で正常に追従を続ける事象を観測した (2026-08-21、
`|q|`=1.000 で角速度とも整合、以降なめらか)。跳躍は「最後に *採用* した値」との差分で
見るので、比較対象が飛ぶ前の姿勢に固定されたままになり、**150 秒間ずっと棄却し続けて
復帰しなかった** (棄却率 96%)。姿勢だけが古い値に貼り付くので、制御は effectively
盲目になる。そこで `stale` に達したら跳躍チェックだけを解除して再同期する
(絶対値で判定できるノルム異常とフルスケール化けは、そのまま弾き続ける)。
"""

from __future__ import annotations

import math
from dataclasses import dataclass


# BNO055 の角速度フルスケール [rad/s]。これ付近の値は読み出し化けとみなす。
GYRO_FULL_SCALE = 35.74


@dataclass
class ImuSample:
    """検査を通った IMU 値。`quat` は (w, x, y, z) の正規化済みクォータニオン。"""

    quat: tuple[float, float, float, float]
    gyro: tuple[float, float, float]


class ImuSanity:
    """IMU サンプルの妥当性を検査し、化けを弾いて直前の有効値を保持する。

    Parameters
    ----------
    max_gyro:
        角速度の上限 [rad/s]。これを超えたら捨てる。ROV が実際に出しうる値より
        十分大きく、かつフルスケール (35.74) より十分小さい値にする。
    max_step_deg:
        1 サンプル間の姿勢跳躍の上限 [deg]。制御周期と機体の最大角速度から決める。
        既定 30° は 50 Hz なら 1500 deg/s 相当で、実運動では到達しない。
    quat_tol:
        クォータニオンのノルムの許容誤差。
    stale_after:
        連続して捨てた回数がこれを超えたら `stale` を True にする。
    """

    def __init__(self, max_gyro: float = 10.0, max_step_deg: float = 30.0,
                 quat_tol: float = 0.01, stale_after: int = 5) -> None:
        self.max_gyro = float(max_gyro)
        self.max_step = math.radians(float(max_step_deg))
        self.quat_tol = float(quat_tol)
        self.stale_after = int(stale_after)

        self.last: ImuSample | None = None
        self.rejected = 0          # 累計の棄却数 (診断用)
        self.accepted = 0
        self.resyncs = 0           # 跳躍チェックを解除して再同期した回数 (診断用)
        self._consecutive = 0

    @property
    def stale(self) -> bool:
        """直近が連続して棄却され、値が古くなっているか。"""
        return self._consecutive > self.stale_after

    @property
    def reject_ratio(self) -> float:
        total = self.accepted + self.rejected
        return self.rejected / total if total else 0.0

    def update(self, quat_wxyz, gyro_xyz) -> tuple[ImuSample | None, str | None]:
        """1 サンプルを検査する。

        Returns
        -------
        (sample, reason)
            `sample` は採用された値、または直前の有効値 (まだ 1 つも無ければ None)。
            `reason` は棄却した理由の文字列、採用時は None。
        """
        reason = self._check(quat_wxyz, gyro_xyz)
        if reason is not None:
            self.rejected += 1
            self._consecutive += 1
            return self.last, reason

        if self.stale:
            self.resyncs += 1
        w, x, y, z = (float(v) for v in quat_wxyz)
        n = math.sqrt(w * w + x * x + y * y + z * z)
        self.last = ImuSample(quat=(w / n, x / n, y / n, z / n),
                              gyro=tuple(float(v) for v in gyro_xyz))
        self.accepted += 1
        self._consecutive = 0
        return self.last, None

    # ------------------------------------------------------------------ 検査
    def _check(self, quat_wxyz, gyro_xyz) -> str | None:
        try:
            w, x, y, z = (float(v) for v in quat_wxyz)
            gx, gy, gz = (float(v) for v in gyro_xyz)
        except (TypeError, ValueError):
            return "値を float にできない"

        for v in (w, x, y, z, gx, gy, gz):
            if math.isnan(v) or math.isinf(v):
                return "NaN/Inf が含まれる"

        n = math.sqrt(w * w + x * x + y * y + z * z)
        if abs(n - 1.0) > self.quat_tol:
            return f"クォータニオンのノルムが不正 (|q|={n:.4f})"

        gmax = max(abs(gx), abs(gy), abs(gz))
        if gmax > self.max_gyro:
            near_fs = abs(gmax - GYRO_FULL_SCALE) < 1.0
            return (f"角速度が上限超過 ({gmax:.2f} rad/s)"
                    + (" — int16 フルスケール相当の化け" if near_fs else ""))

        # `stale` の間は跳躍チェックを行わない。IMU の姿勢基準そのものが飛んだ場合、
        # 飛ぶ前の値と比べ続ける限り永久に復帰できないため (モジュール冒頭の注記)。
        # 復帰までの遅れは stale_after + 1 サンプル (既定 6 = 50 Hz で 120 ms)。
        if self.last is not None and not self.stale:
            step = _angle_between(self.last.quat, (w / n, x / n, y / n, z / n))
            if step > self.max_step:
                return f"姿勢が急変 ({math.degrees(step):.1f} deg/sample)"
        return None


def _angle_between(qa, qb) -> float:
    """2 つの単位クォータニオン間の回転角 [rad]。符号の曖昧さ (q と -q) を吸収する。"""
    dot = sum(a * b for a, b in zip(qa, qb))
    return 2.0 * math.acos(min(1.0, abs(dot)))
