"""perception_node は AUTO の間だけ推論する (infer_only_in_auto)。

検出器は差し替える。手元に torch / umiusi_perception が無くても回る。
"""
import rclpy
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image
from sinsei_umiusi_msgs.msg import RobotState

import umiusi_autonomy.perception_node as pn


class FakeDetector:
    def __init__(self):
        self.calls = 0

    def __call__(self, rgb):
        self.calls += 1
        return []


def _image(t):
    msg = Image()
    msg.header.stamp.sec = t
    msg.height, msg.width, msg.encoding, msg.step = 2, 2, "rgb8", 6
    msg.data = bytes(12)
    return msg


def _state(s):
    msg = RobotState()
    msg.state = s
    return msg


def _make_node():
    node = pn.PerceptionNode()
    node._model_path = "fake.pt"
    det = FakeDetector()
    node._detector = det
    node._sanitise_fn = lambda rgb, dets: dets
    return node, det


def test_infers_only_in_auto():
    rclpy.init()
    try:
        node, det = _make_node()
        t = 100
        # /robot_state が来ていない (core なし) 間は止めない
        node._on_image(_image(t))
        assert det.calls == 1

        for state, expect in ((RobotState.MANUAL, 0), (RobotState.STANDBY, 0),
                              (RobotState.AUTO, 1), (RobotState.MANUAL, 0)):
            node._on_robot_state(_state(state))
            before = det.calls
            t += 1
            node._on_image(_image(t))
            assert det.calls - before == expect, state
        node.destroy_node()
    finally:
        rclpy.shutdown()


def test_can_be_disabled_at_runtime():
    rclpy.init()
    try:
        node, det = _make_node()
        node._on_robot_state(_state(RobotState.MANUAL))
        node._on_image(_image(100))
        assert det.calls == 0
        res = node.set_parameters([Parameter("infer_only_in_auto", value=False)])
        assert res[0].successful
        node._on_image(_image(101))
        assert det.calls == 1
        node.destroy_node()
    finally:
        rclpy.shutdown()



def test_publishes_one_empty_detection_while_gated():
    """起動の段の待ち (wait_perception) が STANDBY のまま 35 s 待たないように。"""
    rclpy.init()
    try:
        node, det = _make_node()
        sent = []
        node._pub.publish = sent.append
        node._on_robot_state(_state(RobotState.STANDBY))
        for t in range(100, 105):
            node._on_image(_image(t))
        assert det.calls == 0
        assert len(sent) == 1 and len(sent[0].detections) == 0
        node.destroy_node()
    finally:
        rclpy.shutdown()
