"""camera_bridge_node の open / read がワーカースレッドで回り、詰まっても executor が止まらないこと。"""
import itertools
import threading
import time

import numpy as np
import pytest

pytest.importorskip("cv2")
rclpy = pytest.importorskip("rclpy")

from rclpy.parameter import Parameter

from umiusi_autonomy import camera_bridge_node as cbn


class FakeCap:
    """open / read を gate (threading.Event) が立つまで止められる偽の VideoCapture。"""
    log = None               # [(op, time.monotonic())]。fixture で空にする
    open_ok = True
    open_gate = None
    read_gate = None
    read_raises = 0          # 残り何回 read で例外を投げるか

    def __init__(self, *args):
        FakeCap.log.append(("open", time.monotonic()))
        if FakeCap.open_gate is not None:
            FakeCap.open_gate.wait()
        self._ok = FakeCap.open_ok

    def isOpened(self):
        return self._ok

    def read(self):
        if FakeCap.read_raises > 0:
            FakeCap.read_raises -= 1
            raise RuntimeError("v4l2 から即時エラー")
        if FakeCap.read_gate is not None:
            FakeCap.read_gate.wait()
        time.sleep(0.01)
        return True, np.zeros((240, 320, 3), np.uint8)

    def release(self):
        pass


class FakeLogger:
    def __init__(self):
        self.errors = []

    def error(self, msg, **_kw):
        self.errors.append(msg)

    def warning(self, *_a, **_kw):
        pass

    def info(self, *_a, **_kw):
        pass


class FakePub:
    def __init__(self):
        self.n = 0

    def publish(self, _msg):
        self.n += 1


@pytest.fixture
def make_node(monkeypatch, tmp_path):
    FakeCap.log = []
    FakeCap.open_ok = True
    FakeCap.open_gate = None
    FakeCap.read_gate = None
    FakeCap.read_raises = 0
    monkeypatch.setattr(cbn.cv2, "VideoCapture", FakeCap)
    nodes = []

    def make(**params):
        overrides = {"shm_socket": str(tmp_path / "cam.sock"), "reconnect_sec": 0.2,
                     "stall_timeout_sec": 0.4, **params}
        args = ["--ros-args"]
        for k, v in overrides.items():
            args += ["-p", f"{k}:={str(v).lower() if isinstance(v, bool) else v}"]
        rclpy.init(args=args)
        logger = FakeLogger()
        # ワーカーは __init__ の中で起動するので、生成前から差し替える
        monkeypatch.setattr(cbn.CameraBridge, "get_logger", lambda self: logger)
        node = cbn.CameraBridge()
        node._pub = FakePub()
        node.log = logger
        nodes.append(node)
        return node

    yield make
    for gate in (FakeCap.open_gate, FakeCap.read_gate):
        if gate is not None:
            gate.set()
    for n in nodes:
        if not getattr(n, "destroyed", False):
            n.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


def spin_for(node, sec):
    end = time.monotonic() + sec
    while time.monotonic() < end:
        rclpy.spin_once(node, timeout_sec=0.02)


def test_開けない間はreconnect_secごとにだけ開き直す(make_node):
    FakeCap.open_ok = False
    node = make_node()
    spin_for(node, 1.0)
    opens = [t for op, t in FakeCap.log if op == "open"]
    assert 3 <= len(opens) <= 7, len(opens)          # 0.2 s 間隔で 1.0 s
    assert min(b - a for a, b in itertools.pairwise(opens)) >= 0.18
    assert node._pub.n == 0


def test_readが返らなくてもexecutorとパラメータは動き_ERRORを出し_戻ればpublishする(make_node):
    node = make_node(publish_compressed=True)
    spin_for(node, 0.5)
    assert node._pub.n > 0

    FakeCap.read_gate = threading.Event()            # 書き手が詰まった
    ticks = []
    node.create_timer(0.05, lambda: ticks.append(1))
    spin_for(node, 1.5)
    assert len(ticks) >= 20                           # executor は止まっていない
    res = node.set_parameters([Parameter("jpeg_quality", value=50)])
    assert res[0].successful and node._jpeg_q == 50
    assert any("read" in m and "返りません" in m for m in node.log.errors)

    n0 = node._pub.n
    FakeCap.read_gate.set()                           # 書き手が戻った
    FakeCap.read_gate = None
    spin_for(node, 0.5)
    assert node._pub.n > n0


def test_止まったワーカーは上限まで作り直し_古い世代の結果は捨てる(make_node):
    node = make_node(max_stalled_workers=2)
    spin_for(node, 0.3)
    FakeCap.read_gate = threading.Event()
    spin_for(node, 3.0)                               # stall 0.4 s を何度も超える
    assert node._gen == 3                             # 初代 + 作り直し 2 本で打ち止め
    assert any("上限" in m for m in node.log.errors)

    FakeCap.read_gate.set()
    FakeCap.read_gate = None
    spin_for(node, 0.5)
    alive = [t for t in threading.enumerate() if t.name.startswith("camera_capture_")]
    assert [t.name for t in alive] == ["camera_capture_3"]   # 古い世代は戻ってきて抜けた


def test_openが返らなくても固まらず_終了できる(make_node):
    FakeCap.open_gate = threading.Event()
    node = make_node()
    ticks = []
    node.create_timer(0.05, lambda: ticks.append(1))
    spin_for(node, 1.0)
    assert len(ticks) >= 15
    assert any("open" in m for m in node.log.errors)
    t0 = time.monotonic()
    node.destroy_node()                               # 止まったワーカーを待ち続けない
    node.destroyed = True
    assert time.monotonic() - t0 < 3.0


def test_readが例外で落ちても作り直してpublishを再開する(make_node):
    node = make_node()
    spin_for(node, 0.3)
    FakeCap.read_raises = 1                           # 詰まりではなく即時の例外でスレッドが落ちる
    spin_for(node, 0.3)
    assert any("落ちました" in m for m in node.log.errors)
    n0 = node._pub.n
    spin_for(node, 1.0)                               # reconnect_sec 0.2 s 後に作り直す
    assert node._gen == 2
    assert node._pub.n > n0
