"""指定トピックに最初のメッセージが来るまで待って終了する。launch の順序ゲート用。

umiusi_stack.sh は control -> autonomy -> rl を sleep 20/35/10 で並べているが、
待っているのは依存関係ではなく起動時の CPU 競合 (Pi で torch を 2 回読む間に
controller_manager と xacro が走る)。sleep は速い機体では無駄に待ち、遅い機体では
足りないので、実際のシグナルで待つ。

launch からは OnProcessExit と組み合わせて使う:

    wait = Node(package="umiusi_autonomy", executable="wait_for_topic",
                arguments=["--topic", "/state/imu", "--timeout", "60"])
    RegisterEventHandler(OnProcessExit(target_action=wait, on_exit=[<次の段>]))

終了コード: 0 = 来た / 1 = timeout / 2 = 引数不正。timeout でも 0 で抜けたい場合は
--allow-timeout (段を止めずに警告だけ出したいとき)。

トピック型は起動時に解決する。まだ誰も publish していない型は解決できないので、
その場合は解決できるまで再試行する (publisher が現れる = 型が判る)。
"""

from __future__ import annotations

import argparse
import sys

import rclpy
from rclpy.node import Node
from rclpy.utilities import remove_ros_args
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy


def _parse(argv):
    p = argparse.ArgumentParser(prog="wait_for_topic", description=__doc__)
    p.add_argument("--topic", required=True)
    p.add_argument("--timeout", type=float, default=60.0, help="秒。0 以下で無制限")
    p.add_argument("--allow-timeout", action="store_true",
                   help="timeout でも 0 で抜ける (段を止めない)")
    p.add_argument("--best-effort", action="store_true",
                   help="BEST_EFFORT で購読する (センサ系の publisher に合わせる)")
    return p.parse_args(argv)


class _Waiter(Node):
    def __init__(self, topic: str, best_effort: bool):
        super().__init__("wait_for_topic")
        self._topic = topic
        self._best_effort = best_effort
        self._sub = None
        self.received = False

    def try_subscribe(self) -> bool:
        """型が解決できたら購読する。まだ publisher が居なければ False。"""
        if self._sub is not None:
            return True
        types = dict(self.get_topic_names_and_types()).get(self._topic)
        if not types:
            return False
        from rosidl_runtime_py.utilities import get_message
        qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            # BEST_EFFORT の購読は RELIABLE / BEST_EFFORT どちらの publisher にも繋がるが、
            # RELIABLE の購読は BEST_EFFORT の publisher に繋がらない。誰が publish するか
            # 分からない用途 (/state/imu は control か sim bridge) では緩いほうを選ぶ。
            # 到達保証は要らない — 1 通来たことが分かればよい
            reliability=(QoSReliabilityPolicy.BEST_EFFORT if self._best_effort
                         else QoSReliabilityPolicy.RELIABLE),
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self._sub = self.create_subscription(
            get_message(types[0]), self._topic, self._on_msg, qos)
        return True

    def _on_msg(self, _msg) -> None:
        self.received = True


def wait(topic: str, timeout: float, best_effort: bool = False, node=None) -> bool:
    """最初のメッセージが来たら True。timeout なら False。"""
    own = node is None
    w = node or _Waiter(topic, best_effort)
    try:
        start = w.get_clock().now().nanoseconds * 1e-9
        while rclpy.ok():
            w.try_subscribe()
            rclpy.spin_once(w, timeout_sec=0.1)
            if w.received:
                return True
            if timeout > 0.0 and (w.get_clock().now().nanoseconds * 1e-9) - start > timeout:
                return False
        return False
    finally:
        if own:
            w.destroy_node()


def main(argv=None) -> int:
    # launch から Node(name=...) で起動されると --ros-args -r __node:=... が付く。
    # argparse に渡す前に落とさないと unrecognized arguments で即死する。
    # OnProcessExit は異常終了でも発火するので、段は「待った」ように見えて素通りする
    rclpy.init(args=sys.argv)
    args = _parse(remove_ros_args(sys.argv)[1:] if argv is None else argv)
    try:
        node = _Waiter(args.topic, args.best_effort)
        ok = wait(args.topic, args.timeout, args.best_effort, node=node)
        if ok:
            node.get_logger().info(f"{args.topic} が来ました")
        else:
            node.get_logger().warning(
                f"{args.topic} が {args.timeout:.0f} s 来ませんでした"
                + (" (--allow-timeout なので続行します)" if args.allow_timeout else ""))
        node.destroy_node()
        return 0 if (ok or args.allow_timeout) else 1
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    sys.exit(main())
