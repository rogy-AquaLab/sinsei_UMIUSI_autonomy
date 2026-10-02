import math

import pytest

from umiusi_common.servo_angle import RAD_SINCE_NS, from_bag, to_wire


@pytest.mark.parametrize("rad, expected", [
    (math.pi / 4, math.pi / 4),
    (2.0, math.pi / 2),
    (-3.0, -math.pi / 2),
])
def test_to_wire_clamps_to_servo_range(rad, expected):
    assert to_wire(rad) == pytest.approx(expected)


def test_bag_before_switch_is_degrees():
    # 2026-09-13 の bag (deg で録られている)
    assert from_bag(45.0, RAD_SINCE_NS - 1) == pytest.approx(math.pi / 4)


def test_bag_after_switch_is_radians():
    assert from_bag(0.5, RAD_SINCE_NS) == pytest.approx(0.5)
