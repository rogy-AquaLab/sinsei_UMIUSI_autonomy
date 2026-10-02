"""ノードの配線: 検出メッセージが止まったら FSM に空の検出が渡ること (B-24)。

FSM (umiusi_perception) は差し替える。手元に umiusi_perception が無くても回る。
"""
from collections import namedtuple

import rclpy
from umiusi_autonomy_msgs.msg import BalloonDetection, BalloonDetectionArray

import umiusi_autonomy.auto_target_generator as atg

FakeDetection = namedtuple(
    "FakeDetection",
    "colour points bbox centroid area_px bearing range_m confidence")


class FakeBehavior:
    def __init__(self):
        self.calls = []

    def step(self, dets, yaw_rate, heading, dt, fresh):
        self.calls.append(list(dets))
        return {"surge": 0.0, "heave": 0.0, "yaw": 0.0}, {}


def test_detections_stop_reaching_fsm_after_timeout(monkeypatch):
    rclpy.init()
    try:
        node = atg.AutoTargetGenerator()
        node.trigger_configure()
        node._behavior = FakeBehavior()
        node._Detection = FakeDetection

        clock = {"t": 100.0}
        monkeypatch.setattr(atg.time, "monotonic", lambda: clock["t"])

        msg = BalloonDetectionArray()
        d = BalloonDetection()
        d.colour = "red"
        d.bbox = [1, 2, 3, 4]
        d.centroid = [1.0, 2.0]
        msg.detections = [d]
        node._on_detections(msg)

        clock["t"] = 100.2
        node._tick()
        clock["t"] = 101.0   # 1 s 何も来ない (カメラが止まった)
        node._tick()

        held, dropped = node._behavior.calls
        assert [x.colour for x in held] == ["red"]
        assert dropped == []
        node.destroy_node()
    finally:
        rclpy.shutdown()
