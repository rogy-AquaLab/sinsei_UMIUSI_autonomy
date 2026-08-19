"""sensor_msgs/Image -> HxWx3 uint8 RGB, with NO cv_bridge dependency.

cv_bridge pulls in a full OpenCV build; the detector only needs a plain RGB numpy array, so this
does the handful of encodings the onboard camera actually emits (rgb8 / bgr8 / rgba8 / bgra8 /
mono8) by hand. Keeping it dependency-light matters for the Raspberry Pi 4 target.
"""

from __future__ import annotations

import numpy as np


def image_to_rgb(msg) -> np.ndarray:
    """Convert a sensor_msgs/Image to an (H, W, 3) uint8 RGB array.

    Raises ValueError on an unsupported encoding so the node can log and skip the frame rather than
    silently mis-colour it (colour is load-bearing for a balloon detector).
    """
    enc = (msg.encoding or "").lower()
    buf = np.frombuffer(bytes(msg.data), dtype=np.uint8)
    h, w = msg.height, msg.width
    if enc in ("rgb8", "bgr8"):
        img = buf.reshape(h, msg.step)[:, : w * 3].reshape(h, w, 3)
        return img[:, :, ::-1].copy() if enc == "bgr8" else img.copy()
    if enc in ("rgba8", "bgra8"):
        img = buf.reshape(h, msg.step)[:, : w * 4].reshape(h, w, 4)[:, :, :3]
        return img[:, :, ::-1].copy() if enc == "bgra8" else img.copy()
    if enc == "mono8":
        gray = buf.reshape(h, msg.step)[:, :w].reshape(h, w, 1)
        return np.repeat(gray, 3, axis=2).copy()
    raise ValueError(
        f"unsupported image encoding {msg.encoding!r}; supported: rgb8/bgr8/rgba8/bgra8/mono8"
    )
