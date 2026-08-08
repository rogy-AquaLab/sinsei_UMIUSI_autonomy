"""Shared arm / disarm (e-stop) state for the direct-drive autonomy nodes.

A node that commands the thrusters wires an ``ArmState`` and, while disarmed, must stop driving and
assert a DETACH — publish ``ThrusterOutput`` with ``runnable.esc = runnable.servo = false`` and zero
output, which the control stack resolves to esc/servo *not allowed* (the hardware-level detach). This
is the emergency-stop path for the direct-override control loop; core's power/mode gating is the other.

Interface exposed on the owning node:
  * ``~/estop`` (std_msgs/Bool)  — ``data: true`` DISARMs immediately; ``data: false`` re-arms.
    Latched (transient_local, reliable) QoS: a node that (re)starts while an e-stop is asserted picks
    up the last value and comes up DISARMED, provided the publisher is still alive.
  * ``~/arm``   (std_srvs/SetBool) — ``data: true`` arms, ``data: false`` disarms (programmatic).
"""

from __future__ import annotations

from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import Bool
from std_srvs.srv import SetBool

# Latched so a late-joining / restarted node inherits the last e-stop state (reliable + keep-last-1).
ESTOP_QOS = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)


class ArmState:
    def __init__(self, node, on_disarm, *, start_armed=True,
                 estop_topic="~/estop", arm_service="~/arm"):
        self._node = node
        self._on_disarm = on_disarm          # called on every disarm transition (publish the detach)
        self.armed = bool(start_armed)
        self._sub = node.create_subscription(Bool, estop_topic, self._on_estop, ESTOP_QOS)
        self._srv = node.create_service(SetBool, arm_service, self._on_arm)
        node.get_logger().info(
            f"arm state: {'ARMED' if self.armed else 'DISARMED'} "
            f"(e-stop on '{estop_topic}', arm service '{arm_service}')")

    def disarm(self, reason=""):
        was = self.armed
        self.armed = False
        if was:
            self._node.get_logger().warning(f"DISARMED{f' ({reason})' if reason else ''}")
        self._on_disarm()

    def arm(self):
        if not self.armed:
            self._node.get_logger().info("ARMED")
        self.armed = True

    def _on_estop(self, msg):
        if msg.data:
            self.disarm("e-stop")
        else:
            self.arm()

    def _on_arm(self, req, resp):
        if req.data:
            self.arm()
        else:
            self.disarm("arm service")
        resp.success = True
        resp.message = "armed" if self.armed else "disarmed"
        return resp
