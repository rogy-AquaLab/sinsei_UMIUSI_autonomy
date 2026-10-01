"""サーボ角の単位 (rad) と、古い bag の読み替え。

規約:
  * ThrusterOutput.angle / ThrusterState の角度は rad、範囲 [-pi/2, pi/2]
    (control 91cd49d 以降。範囲外は CAN フレームが作られず、サーボだけ動かない)
  * それより前に録った bag の角度は deg。録った日時で読み替える
"""
import math

HALF_PI = math.pi / 2.0

# control が rad に切り替わった日 (91cd49d, 2026-09-14 JST)。これより前の bag は deg
RAD_SINCE_NS = 1789311600 * 10**9  # 2026-09-14T00:00:00+09:00


def to_wire(rad: float) -> float:
    """送る直前の角度。可動範囲に収める。"""
    return max(-HALF_PI, min(HALF_PI, float(rad)))


def from_bag(value: float, bag_start_ns: int) -> float:
    """bag に入っているサーボ角を rad で返す。"""
    return math.radians(value) if bag_start_ns < RAD_SINCE_NS else float(value)
