"""teleop_keyboard — keyboard teleop for the RL attitude(-velocity) controller (experiments).

Publishes a target attitude (geometry_msgs/Quaternion) + target velocity (geometry_msgs/Vector3,
target-body frame) to ``rl_attitude_node`` — designed for 3-D motion (separate keys per axis), unlike
teleop_twist_keyboard. Includes an EMERGENCY STOP that both signals the controller to disarm AND
directly detaches the thrusters (independent of the controller staying alive).

Run in its own terminal (it needs the keyboard):

    ros2 run umiusi_autonomy teleop_keyboard

Keys
  velocity (body frame, m/s)      attitude target (deg)          safety
    w / s   +x / -x  (forward)      i / k   pitch +/-              SPACE  zero velocity (hold attitude)
    a / d   +y / -y  (sway)         j / l   yaw   +/-              t      reset attitude to upright
    r / f   +z / -z  (heave)        u / o   roll  +/-              x      EMERGENCY STOP (disarm+detach)
                                                                   z      ARM / clear e-stop
                                                                   q      quit (disarms on exit)
"""

from __future__ import annotations

import math
import select
import sys
import termios
import tty

import rclpy
from geometry_msgs.msg import Quaternion, Vector3
from rclpy.node import Node
from sinsei_umiusi_msgs.msg import ThrusterOutput, ThrusterRunnable
from std_msgs.msg import Bool

POSITIONS = ("lf", "lb", "rb", "rf")
CMD_PREFIX = "/cmd/direct/thruster_controller/output_"

HELP = __doc__[__doc__.index("Keys"):]


def rpy_to_quat(roll, pitch, yaw):
    """Intrinsic Z-Y-X (yaw, pitch, roll) euler [rad] -> quaternion (w, x, y, z)."""
    cr, sr = math.cos(roll / 2), math.sin(roll / 2)
    cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
    cy, sy = math.cos(yaw / 2), math.sin(yaw / 2)
    return (cr * cp * cy + sr * sp * sy,
            sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy,
            cr * cp * sy - sr * sp * cy)


class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__("teleop_keyboard")
        self.declare_parameter("attitude_topic", "/rl_attitude_node/target_attitude")
        self.declare_parameter("velocity_topic", "/rl_attitude_node/velocity_cmd")
        self.declare_parameter("estop_topic", "/rl_attitude_node/estop")
        self.declare_parameter("vel_step", 0.05)      # m/s per key
        self.declare_parameter("vel_max", 0.4)        # clamp
        self.declare_parameter("ang_step_deg", 5.0)   # deg per key

        self._vel_step = float(self.get_parameter("vel_step").value)
        self._vel_max = float(self.get_parameter("vel_max").value)
        self._ang_step = float(self.get_parameter("ang_step_deg").value)

        self._pub_att = self.create_publisher(Quaternion, self.get_parameter("attitude_topic").value, 10)
        self._pub_vel = self.create_publisher(Vector3, self.get_parameter("velocity_topic").value, 10)
        self._pub_estop = self.create_publisher(Bool, self.get_parameter("estop_topic").value, 10)
        self._detach_pubs = {p: self.create_publisher(ThrusterOutput, CMD_PREFIX + p, 10) for p in POSITIONS}

        self._v = [0.0, 0.0, 0.0]           # target velocity (body frame)
        self._rpy = [0.0, 0.0, 0.0]         # target attitude euler (deg): roll, pitch, yaw

    # ---- command emission ----
    def _publish_setpoint(self):
        self._pub_vel.publish(Vector3(x=self._v[0], y=self._v[1], z=self._v[2]))
        w, x, y, z = rpy_to_quat(*(math.radians(a) for a in self._rpy))
        self._pub_att.publish(Quaternion(x=x, y=y, z=z, w=w))

    def estop(self, engage: bool):
        self._pub_estop.publish(Bool(data=engage))
        if engage:
            self._v = [0.0, 0.0, 0.0]
            for p in POSITIONS:                       # independent hard detach (don't rely on the node)
                out = ThrusterOutput()
                out.runnable = ThrusterRunnable(esc=False, servo=False)
                out.duty_cycle = 0.0
                out.angle = 0.0
                self._detach_pubs[p].publish(out)

    def _clamp_v(self, i, d):
        self._v[i] = max(-self._vel_max, min(self._vel_max, self._v[i] + d))

    def handle(self, key: str) -> bool:
        """Apply a keypress; return False to quit."""
        s, a = self._vel_step, self._ang_step
        if key in ("q", "\x03"):        # q or Ctrl-C
            return False
        elif key == "w":
            self._clamp_v(0, s)
        elif key == "s":
            self._clamp_v(0, -s)
        elif key == "a":
            self._clamp_v(1, s)
        elif key == "d":
            self._clamp_v(1, -s)
        elif key == "r":
            self._clamp_v(2, s)
        elif key == "f":
            self._clamp_v(2, -s)
        elif key == "u":
            self._rpy[0] += a
        elif key == "o":
            self._rpy[0] -= a
        elif key == "i":
            self._rpy[1] += a
        elif key == "k":
            self._rpy[1] -= a
        elif key == "j":
            self._rpy[2] += a
        elif key == "l":
            self._rpy[2] -= a
        elif key == " ":
            self._v = [0.0, 0.0, 0.0]
        elif key == "t":
            self._rpy = [0.0, 0.0, 0.0]
        elif key == "x":
            self.estop(True)
            self.get_logger().warning("EMERGENCY STOP — disarmed + thrusters detached (press 'z' to re-arm)")
            return True
        elif key == "z":
            self.estop(False)
            self.get_logger().info("ARMED (e-stop cleared)")
            return True
        else:
            return True                 # unknown key: ignore, don't republish
        self._publish_setpoint()
        self.get_logger().info(
            f"v(body)=[{self._v[0]:+.2f},{self._v[1]:+.2f},{self._v[2]:+.2f}] m/s  "
            f"rpy=[{self._rpy[0]:+.0f},{self._rpy[1]:+.0f},{self._rpy[2]:+.0f}] deg")
        return True


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboard()
    settings = termios.tcgetattr(sys.stdin)
    print(HELP)
    print("teleop_keyboard ready. focus this terminal and press keys.\n")
    try:
        while True:
            tty.setraw(sys.stdin.fileno())
            r, _, _ = select.select([sys.stdin], [], [], 0.1)
            key = sys.stdin.read(1) if r else ""
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
            if key and not node.handle(key):
                break
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.estop(True)          # disarm + detach on exit
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
