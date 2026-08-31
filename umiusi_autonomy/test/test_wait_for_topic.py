"""wait_for_topic の単体テスト。

launch の順序ゲートなので、外し方を 2 つとも見る:
  * 来たのに待ち続ける -> 起動が進まない
  * 来ていないのに抜ける -> sleep を消した意味が無くなり、CPU 競合が戻る
"""
import threading

import pytest

rclpy = pytest.importorskip("rclpy")

from std_msgs.msg import Bool  # noqa: E402

from umiusi_autonomy.wait_for_topic import _parse, wait  # noqa: E402

TOPIC = "/test_wait_for_topic/ping"


@pytest.fixture
def ros():
    rclpy.init()
    yield
    rclpy.try_shutdown()


def _publisher(node_name, topic, stop):
    """停止フラグが立つまで 20 Hz で publish し続けるスレッドを返す。

    publish に spin は要らない。ここで spin_once するとテスト本体の spin と
    衝突して "Executor is already spinning" になる。
    """
    import time

    from rclpy.node import Node
    node = Node(node_name)
    pub = node.create_publisher(Bool, topic, 1)

    def loop():
        while not stop.is_set() and rclpy.ok():
            pub.publish(Bool(data=True))
            time.sleep(0.05)
        node.destroy_node()

    t = threading.Thread(target=loop, daemon=True)
    t.start()
    return t


def test_メッセージが来たら_Trueで返る(ros):
    stop = threading.Event()
    t = _publisher("test_pub_ok", TOPIC, stop)
    try:
        assert wait(TOPIC, timeout=10.0) is True
    finally:
        stop.set()
        t.join(timeout=2.0)


def test_誰も出していなければtimeoutでFalse(ros):
    # publisher を立てない。型が解決できないので購読もできない = 来ない
    assert wait("/test_wait_for_topic/never", timeout=1.0) is False


def test_publisherが遅れて現れても拾う(ros):
    """control の起動待ちがこの形。購読開始時点では publisher がまだ居ない。"""
    stop = threading.Event()
    late = []

    def start_late():
        import time
        time.sleep(1.0)
        late.append(_publisher("test_pub_late", TOPIC + "_late", stop))

    threading.Thread(target=start_late, daemon=True).start()
    try:
        assert wait(TOPIC + "_late", timeout=15.0) is True
    finally:
        stop.set()
        for t in late:
            t.join(timeout=2.0)


# --- 引数 -------------------------------------------------------------------

def test_topicは必須():
    with pytest.raises(SystemExit):
        _parse([])


def test_既定のtimeoutは60秒():
    assert _parse(["--topic", "/x"]).timeout == 60.0


def test_allow_timeoutは既定で無効():
    assert _parse(["--topic", "/x"]).allow_timeout is False
    assert _parse(["--topic", "/x", "--allow-timeout"]).allow_timeout is True
