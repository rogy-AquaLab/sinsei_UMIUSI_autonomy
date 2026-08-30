"""ArmState の単体テスト。移設にあわせて新設 (それまでテストが 1 つも無かった)。

ここは緊急停止の経路なので、契約を固定しておく価値が高い:
  * disarm したら 毎回 detach コールバックを呼ぶ (「既に disarmed だから何もしない」は危険。
    ノードは tick ごとに detach を打ち直す設計で、その打ち直しがここを通る)
  * arm は状態を上げるだけで detach は呼ばない
  * ~/estop の true/false と ~/arm サービスの両方から同じ状態機械に入る
"""
import pytest

rclpy = pytest.importorskip("rclpy")

from rclpy.qos import DurabilityPolicy  # noqa: E402
from std_msgs.msg import Bool  # noqa: E402
from std_srvs.srv import SetBool  # noqa: E402

from umiusi_common.arm import ArmState  # noqa: E402


class _Node:
    """ArmState が触るぶんだけの偽ノード。rclpy のノードは立てない。"""

    def __init__(self):
        self.subs, self.srvs, self.logs = [], [], []

    def create_subscription(self, msg_type, topic, cb, qos):
        self.subs.append((msg_type, topic, cb, qos))
        return object()

    def create_service(self, srv_type, name, cb):
        self.srvs.append((srv_type, name, cb))
        return object()

    def get_logger(self):
        node = self

        class _L:
            def info(self, m):
                node.logs.append(("info", m))

            def warning(self, m):
                node.logs.append(("warn", m))
        return _L()


def _make(start_armed=True):
    n = _Node()
    calls = []
    a = ArmState(n, lambda: calls.append(1), start_armed=start_armed)
    return n, a, calls


def test_既定は武装状態で立ち上がる():
    _, a, calls = _make()
    assert a.armed is True
    assert calls == [], "起動しただけで detach を打ってはいけない"


def test_start_armed_falseなら解除状態で立ち上がる():
    _, a, _ = _make(start_armed=False)
    assert a.armed is False


def test_disarmは毎回detachを呼ぶ():
    """既に解除済みでも呼ぶ。 ノードは tick ごとに detach を打ち直す設計で、
    「もう解除済みだから省略」にすると指令が残ったままになりうる。"""
    _, a, calls = _make()
    a.disarm("test")
    a.disarm("test")
    a.disarm("test")
    assert a.armed is False
    assert len(calls) == 3, f"detach が {len(calls)} 回しか呼ばれていない"


def test_armはdetachを呼ばない():
    _, a, calls = _make(start_armed=False)
    a.arm()
    assert a.armed is True
    assert calls == []


def test_estopトピックで解除と復帰ができる():
    n, a, calls = _make()
    (_, topic, cb, qos) = n.subs[0]
    assert topic == "~/estop"
    # 同一性ではなく中身を見る。 qos is ESTOP_QOS だけだと、ESTOP_QOS の durability を
    # VOLATILE に落とす変更が素通りする (= 守りたい性質そのものが壊れても気付けない)
    assert qos.durability == DurabilityPolicy.TRANSIENT_LOCAL, \
        "latch されていないと、e-stop 中に再起動したノードが武装状態で上がってくる"
    assert qos.depth >= 1
    cb(Bool(data=True))
    assert a.armed is False and len(calls) == 1
    cb(Bool(data=False))
    assert a.armed is True


def test_armサービスで解除と復帰ができる():
    n, a, calls = _make()
    (srv_type, name, cb) = n.srvs[0]
    assert srv_type is SetBool and name == "~/arm"
    resp = cb(SetBool.Request(data=False), SetBool.Response())
    assert a.armed is False and resp.success is True and resp.message == "disarmed"
    assert len(calls) == 1
    resp = cb(SetBool.Request(data=True), SetBool.Response())
    assert a.armed is True and resp.message == "armed"


def test_解除の理由がログに出る():
    """プールで「なぜ止まったか」を後から追えるようにするための契約。"""
    n, a, _ = _make()
    a.disarm("e-stop")
    assert any(lvl == "warn" and "e-stop" in m for lvl, m in n.logs), n.logs
