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


def test_balloon_on_the_right_turns_right_in_rep103():
    """FSM の + yaw は画像の右へ、control の AttitudeTarget.yaw_rate は REP-103 (+ = 左回り)。

    右前の風船には yaw_rate < 0 (右回り) を出すこと。umiusi_sim の閉ループ再現で、符号を反転しないと
    風船から離れていった (10/08)。実機では未確認 — field_card 6.
    """
    import os

    import yaml
    behavior = pytest.importorskip("umiusi_perception.autonomy.behavior")
    from umiusi_perception.balloon_detector import Detection

    cfg = os.path.join(os.path.dirname(__file__), "..", "config", "competition.yaml")
    with open(cfg) as f:
        scale = yaml.safe_load(f)["auto_target_generator"]["ros__parameters"]["yaw_rate_scale"]

    b = behavior.BalloonBehavior(frame_h=240, frame_w=320, fovy_deg=60.0, dt=0.02)
    det = Detection(colour="yellow", points=10, bbox=(250, 100, 280, 140), centroid=(265.0, 120.0),
                    area_px=1200, bearing=(0.35, 0.0), range_m=2.0, confidence=0.6)   # 画像の右
    cmds = [b.step([det], 0.0, dt=0.02, fresh=True)[0] for _ in range(10)]
    assert cmds[-1]["yaw"] > 0.0                       # FSM は右へ回ろうとする
    _, _, yaw_rate = to_control_setpoint(cmds[-1], 1.0, scale)
    assert yaw_rate < 0.0                              # REP-103 で右回り
