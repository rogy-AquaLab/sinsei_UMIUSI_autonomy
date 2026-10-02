import pytest

from umiusi_autonomy.auto_target_generator import neutral_attitude_target, to_control_setpoint


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
