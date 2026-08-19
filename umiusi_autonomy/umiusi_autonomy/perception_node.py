"""perception_node — onboard balloon detection, a THIN rclpy wrapper around the shared library.

Subscribes the onboard camera (sensor_msgs/Image), runs the learned detector
(``umiusi_perception.learned_detector``) followed by the near-range red/blue colour
re-confirmation (``umiusi_perception.sanitise_near_colours``) — EXACTLY the perception half of
``tools/autonomy_run`` — and publishes the per-frame detections as ``BalloonDetectionArray``. All
detection logic lives in the ROS-free ``umiusi_perception`` package; this node only does topic plumbing and
message conversion, so the same detector runs bit-identically in sim and on the robot.

The heavy imports (torch, umiusi_perception) are deferred until the first image so ``colcon build`` and
``--help`` do not require them; on the Pi they are imported once at startup.

Parameters
----------
model_path      : learned detector checkpoint (.pt). REQUIRED.
image_topic     : onboard camera topic (default /front_cam/image_raw).
detections_topic: output topic (default ~/detections).
conf_thresh     : detector confidence floor (default: the checkpoint's stored value).
input_size      : detector square input (default: the checkpoint's stored value).
fovy_deg        : camera vertical FOV, must match the physical camera (default 60.0).
max_rate_hz     : cap the detector rate; frames arriving faster are dropped (default 10 Hz, 0 = no cap).
sanitise_near   : run the near red/blue colour re-confirmation (default True).
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image

from umiusi_autonomy_msgs.msg import BalloonDetection, BalloonDetectionArray

from umiusi_autonomy.image_convert import image_to_rgb


class PerceptionNode(Node):
    def __init__(self):
        super().__init__("perception_node")
        self.declare_parameter("model_path", "")
        self.declare_parameter("image_topic", "/front_cam/image_raw")
        self.declare_parameter("detections_topic", "~/detections")
        self.declare_parameter("conf_thresh", -1.0)   # <0 -> use the checkpoint's stored floor
        self.declare_parameter("input_size", 0)        # 0 -> use the checkpoint's stored size
        self.declare_parameter("fovy_deg", 60.0)
        self.declare_parameter("max_rate_hz", 10.0)
        self.declare_parameter("sanitise_near", True)

        self._model_path = self.get_parameter("model_path").value
        self._fovy = float(self.get_parameter("fovy_deg").value)
        self._sanitise = bool(self.get_parameter("sanitise_near").value)
        rate = float(self.get_parameter("max_rate_hz").value)
        self._min_period = (1.0 / rate) if rate > 0 else 0.0
        self._last_stamp = None

        self._detector = None       # lazily loaded on the first frame (defer torch import)
        self._sanitise_fn = None

        image_topic = self.get_parameter("image_topic").value
        det_topic = self.get_parameter("detections_topic").value
        self._pub = self.create_publisher(BalloonDetectionArray, det_topic, 10)
        self._sub = self.create_subscription(Image, image_topic, self._on_image, 1)

        if not self._model_path:
            self.get_logger().error(
                "parameter 'model_path' is empty — set it to a learned detector .pt checkpoint")
        self.get_logger().info(
            f"perception_node: image='{image_topic}' -> detections='{det_topic}' "
            f"(fovy={self._fovy:.0f}deg, max_rate={rate:.0f}Hz, sanitise_near={self._sanitise})")

    def _ensure_detector(self):
        """Load the detector on first use (defers the torch/umiusi_perception import off the build path)."""
        if self._detector is not None:
            return True
        try:
            from umiusi_perception.learned_detector import load_learned_detector
            from umiusi_perception.tracker import sanitise_near_colours
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(
                f"cannot import the detector from umiusi_perception ({type(e).__name__}: {e}); "
                "is the umiusi_perception wheel installed (pip install .../packages/perception)?")
            return False
        conf = self.get_parameter("conf_thresh").value
        size = self.get_parameter("input_size").value
        try:
            self._detector = load_learned_detector(
                self._model_path,
                input_size=(int(size) if int(size) > 0 else None),
                conf_thresh=(float(conf) if float(conf) >= 0.0 else None),
                fovy_deg=self._fovy,
            )
        except Exception as e:  # noqa: BLE001
            self.get_logger().error(f"failed to load detector '{self._model_path}': "
                                    f"{type(e).__name__}: {e}")
            return False
        self._sanitise_fn = sanitise_near_colours
        self.get_logger().info(f"detector loaded from '{self._model_path}'")
        return True

    def _on_image(self, msg: Image):
        # rate cap: drop frames that arrive faster than max_rate_hz (realistic Pi-4 detector timing).
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self._min_period > 0.0 and self._last_stamp is not None \
                and (stamp - self._last_stamp) < self._min_period:
            return
        if not self._model_path or not self._ensure_detector():
            return
        try:
            rgb = image_to_rgb(msg)
        except ValueError as e:
            self.get_logger().warn(str(e), throttle_duration_sec=5.0)
            return
        dets = self._detector(rgb)
        if self._sanitise:
            dets = self._sanitise_fn(rgb, dets)
        self._last_stamp = stamp
        self._pub.publish(self._to_msg(msg.header, dets))

    @staticmethod
    def _to_msg(header, dets) -> BalloonDetectionArray:
        out = BalloonDetectionArray()
        out.header = header
        for d in dets:
            m = BalloonDetection()
            m.colour = d.colour
            m.points = int(d.points)
            m.azimuth = float(d.bearing[0])
            m.elevation = float(d.bearing[1])
            m.range_m = float(d.range_m)
            m.confidence = float(d.confidence)
            m.bbox = [int(x) for x in d.bbox]
            m.centroid = [float(d.centroid[0]), float(d.centroid[1])]
            m.area_px = int(d.area_px)
            out.detections.append(m)
        return out


def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
