import pytest

from umiusi_autonomy.auto_target_generator import (
    live_detections,
    neutral_attitude_target,
    to_control_setpoint,
)


def test_surge_follows_ui_convention_by_default():
    vx, vz, yaw_rate = to_control_setpoint({"surge": 0.4, "heave": 0.0, "yaw": 0.0}, 1.0, 1.0)
    assert vx == pytest.approx(0.4)


def test_surge_sign_flips_forward():
    vx, _, _ = to_control_setpoint({"surge": 0.4, "heave": 0.0, "yaw": 0.0}, -1.0, 1.0)
    assert vx == pytest.approx(-0.4)


def test_yaw_is_scaled_to_rate_and_heave_passes_through():
    _, vz, yaw_rate = to_control_setpoint({"surge": 0.0, "heave": -0.3, "yaw": 0.5}, 1.0, 0.8)
    assert vz == pytest.approx(-0.3)
    assert yaw_rate == pytest.approx(0.4)


def test_commands_are_clipped_to_normalized_range():
    vx, vz, yaw_rate = to_control_setpoint({"surge": 2.0, "heave": -5.0, "yaw": 3.0}, 1.0, 1.0)
    assert (vx, vz, yaw_rate) == pytest.approx((1.0, -1.0, 1.0))


def test_neutral_attitude_target_is_level_rate_mode():
    msg = neutral_attitude_target()
    assert (msg.attitude.x, msg.attitude.y, msg.attitude.z, msg.attitude.w) == (0.0, 0.0, 0.0, 1.0)
    assert msg.yaw_rate == 0.0
    assert msg.hold_yaw is False


def test_no_detection_message_yet_means_nothing_seen():
    assert live_detections(["d"], None, 10.0, 0.5) == []


def test_recent_detections_are_held_between_perception_frames():
    # perception は最大 10 Hz、制御は 50 Hz なので、0.1 s 程度は同じ検出を使い回す
    assert live_detections(["d"], 10.0, 10.1, 0.5) == ["d"]


def test_detections_are_dropped_when_messages_stop():
    # 2026-10-01: カメラが止まったあと、最後の検出で居ない風船を追い続けた
    assert live_detections(["d"], 10.0, 10.6, 0.5) == []


def test_timeout_zero_disables_dropping():
    assert live_detections(["d"], 10.0, 100.0, 0.0) == ["d"]


def test_fsm_tuning_overrides_module_constants_and_ki_heave():
    import types

    from umiusi_autonomy.auto_target_generator import apply_fsm_tuning

    module = types.ModuleType("fake_behavior_module")
    module.RAM_SURGE, module.RAM_MAX_STEPS = 0.26, 85

    class Behavior:
        ki_heave = 0.0
    Behavior.__module__ = module.__name__
    import sys
    sys.modules[module.__name__] = module
    try:
        b = Behavior()
        apply_fsm_tuning(b, ram_surge=0.6, ram_max_steps=200, ki_heave=0.3)
        assert (module.RAM_SURGE, module.RAM_MAX_STEPS, b.ki_heave) == (0.6, 200, 0.3)
        apply_fsm_tuning(b, ram_surge=0.26, ram_max_steps=85, ki_heave=0.0)   # 元に戻せる
        assert (module.RAM_SURGE, module.RAM_MAX_STEPS, b.ki_heave) == (0.26, 85, 0.0)
    finally:
        del sys.modules[module.__name__]


def test_fsm_tuning_tolerates_old_wheel_without_ki_heave():
    import sys
    import types

    from umiusi_autonomy.auto_target_generator import apply_fsm_tuning

    module = types.ModuleType("old_behavior_module")

    class Behavior:
        pass
    Behavior.__module__ = module.__name__
    sys.modules[module.__name__] = module
    try:
        b = Behavior()
        apply_fsm_tuning(b, ram_surge=0.6, ram_max_steps=200, ki_heave=0.3)
        assert not hasattr(b, "ki_heave")
        assert module.RAM_SURGE == 0.6
    finally:
        del sys.modules[module.__name__]
