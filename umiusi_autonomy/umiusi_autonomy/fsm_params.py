"""競技 FSM (umiusi_perception の BalloonBehavior) の調整値を ROS パラメータ `fsm.*` として出す。

値の置き場所は config/competition.yaml の 1 箇所。ここは名前・単位・FSM 側の実体の対応表だけを持つ。
FSM の調整値の大半は behavior.py のモジュール定数で、実行時に名前で引かれるので、モジュールの属性を
書き換えれば次の周期から効く。

規約:
  * `*_deg` は度で受け、FSM にはラジアンで入れる
  * `*_steps` / `*_frames` は FSM の周期数。実周期は control_hz (50 Hz) ではなく CPU 次第
    (2026-10-03 実機の AUTO で 26 Hz)。秒で考えるなら実周期で割ること
  * yaml に無い名前は FSM の既定値のまま (宣言時に FSM から読む)
"""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass


@dataclass(frozen=True)
class FsmParam:
    name: str          # ROS パラメータ名 (fsm. の後ろ)
    target: str        # FSM 側の名前
    kind: str          # "float" | "int" | "deg"
    on_instance: bool = False   # True: BalloonBehavior のフィールド / False: behavior.py のモジュール定数


def _p(name, target, kind="float", on_instance=False):
    return FsmParam(name, target, kind, on_instance)


FSM_PARAMS: tuple[FsmParam, ...] = (
    # 探索 (その場旋回 -> 少し前進 -> また旋回)
    _p("search_yaw", "SEARCH_YAW"),
    _p("search_surge", "SEARCH_SURGE"),
    _p("scan_heave", "SCAN_HEAVE"),
    _p("scan_rate", "SCAN_RATE"),
    _p("translate_steps", "TRANSLATE_STEPS", "int"),
    _p("sweep_timeout_s", "SWEEP_TIMEOUT_S"),
    # 目標の選び方・諦め方
    _p("max_target_range", "MAX_TARGET_RANGE"),
    _p("preempt_margin", "PREEMPT_MARGIN"),
    _p("path_clear_radius", "PATH_CLEAR_RADIUS"),
    _p("max_attempts", "MAX_ATTEMPTS", "int"),
    _p("max_pursuit_steps", "MAX_PURSUIT_STEPS", "int"),
    _p("giveup_fails", "GIVEUP_FAILS", "int"),
    _p("suppress_steps", "SUPPRESS_STEPS", "int"),
    # 接近
    _p("speed_cap", "SPEED_CAP"),
    _p("kp_yaw", "KP_YAW"),
    _p("kd_yaw", "KD_YAW"),
    _p("kp_heave", "KP_HEAVE"),
    _p("ki_heave", "ki_heave", on_instance=True),
    _p("heave_bias_max", "heave_bias_max", on_instance=True),
    _p("face_tol_deg", "FACE_TOL", "deg"),
    _p("el_full_surge_deg", "EL_FULL_SURGE", "deg"),
    _p("el_zero_surge_deg", "EL_ZERO_SURGE", "deg"),
    _p("el_min_surge", "EL_MIN_SURGE"),
    # 正対 (近づいたら中央に合わせる)
    _p("align_bbox", "ALIGN_BBOX"),
    _p("align_creep", "ALIGN_CREEP"),
    _p("align_timeout", "ALIGN_TIMEOUT", "int"),
    _p("centre_az_deg", "CENTRE_AZ", "deg"),
    _p("centre_el_deg", "CENTRE_EL", "deg"),
    _p("settle_steps", "SETTLE_STEPS", "int"),
    # 突進
    _p("ram_commit_bbox", "RAM_COMMIT_BBOX"),
    _p("commit_el_deg", "COMMIT_EL", "deg"),
    _p("ram_surge", "RAM_SURGE"),
    _p("ram_max_steps", "RAM_MAX_STEPS", "int"),
    _p("lunge_steps", "LUNGE_STEPS", "int"),
    _p("aim_above_deg", "AIM_EL_BIAS", "deg"),
    _p("pass_peak_bbox", "PASS_PEAK_BBOX"),
    _p("pass_drop_frac", "PASS_DROP_FRAC"),
    # 外したあとの後退と、割れたかの確認
    _p("recover_surge", "RECOVER_SURGE"),
    _p("recover_steps", "RECOVER_STEPS", "int"),
    _p("confirm_frames", "CONFIRM_FRAMES", "int"),
    _p("confirm_surge", "CONFIRM_SURGE"),
    _p("confirm_min_peak", "CONFIRM_MIN_PEAK"),
    _p("confirm_edge_deg", "CONFIRM_EDGE", "deg"),
    # 青 (減点) の回避
    _p("avoid_az_deg", "AVOID_AZ", "deg"),
    _p("avoid_range", "AVOID_RANGE"),
    _p("avoid_yaw", "AVOID_YAW"),
)

PREFIX = "fsm."


def _holder(behavior, p: FsmParam):
    return behavior if p.on_instance else sys.modules[type(behavior).__module__]


def missing(behavior) -> list[str]:
    """この FSM (wheel の版) に無い調整値の名前。古い wheel では一部が無い。"""
    return [p.name for p in FSM_PARAMS if not hasattr(_holder(behavior, p), p.target)]


def current_value(behavior, p: FsmParam):
    """FSM が今持っている値を、ROS パラメータの単位で返す。"""
    v = getattr(_holder(behavior, p), p.target)
    if p.kind == "deg":
        return math.degrees(float(v))
    return int(v) if p.kind == "int" else float(v)


def apply(behavior, p: FsmParam, value) -> None:
    v = math.radians(float(value)) if p.kind == "deg" else (int(value) if p.kind == "int" else float(value))
    setattr(_holder(behavior, p), p.target, v)


def by_ros_name(name: str) -> FsmParam | None:
    if not name.startswith(PREFIX):
        return None
    key = name[len(PREFIX):]
    return next((p for p in FSM_PARAMS if p.name == key), None)


def measured_dt(prev: float | None, now: float, nominal: float) -> float:
    """FSM に渡す 1 周期の実時間 [s]。

    固定の 1/control_hz を渡すと、CPU が詰まってタイマが遅れた分だけ FSM の時間が遅れる
    (10/03 実機: 50 Hz のつもりが 26 Hz、探索の 1 周が実際は 1.9 周)。
    初回は nominal、止まっていた後の 1 周期で積分が跳ねないよう 5 周期分で頭打ちにする。
    """
    if prev is None:
        return nominal
    return min(max(now - prev, 0.0), 5.0 * nominal)
