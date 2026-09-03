"""launch ファイルの共通部品。

autonomy.launch.py と core_autonomy.launch.py はどちらもカメラブリッジを起動する。
同じ Node ブロックを 2 箇所に書くと 片方だけ直る (IMU の扱いを ImuSource に寄せたのと
同じ構図)。launch ディレクトリは Python パッケージではないので、共有するものは
インストールされるこのパッケージ側に置く。
"""

from __future__ import annotations

from launch.conditions import IfCondition
from launch_ros.actions import Node


def camera_bridge_node(*, condition, rtsp_url, image_topic, record_vision="false") -> Node:
    """RTSP -> ROS Image のブリッジ。

    実機カメラ (gst_camera_node) は RTSP に流すだけで ROS トピックを出さないので、
    perception にはこれが必要。無いと画像が 1 枚も来ず FSM が SEARCH から出られない
    (known_issues A-18)。

    record_vision を true にすると圧縮画像 (<image_topic>/compressed) も出す。
    視覚での位置固定を作るための素材集め用 — record_run.sh --vision とセットで使う。
    レートを絞るのは JPEG エンコードが perception と同じ CPU を食うため (docs/logging.md)。
    """
    return Node(
        package="umiusi_autonomy",
        executable="camera_bridge_node",
        name="camera_bridge_node",
        output="screen",
        condition=IfCondition(condition),
        parameters=[{
            "rtsp_url": rtsp_url,
            "image_topic": image_topic,
            "width": 320, "height": 240,   # autonomy.yaml の frame_w/frame_h に合わせる
            "max_rate_hz": 0.0,            # 制限をかけると取りこぼす (実測) — カメラ側で絞ること
            "auto_rate": False,            # AIMD 追従は実験的。既定は無効
            "publish_compressed": record_vision,
            "compressed_max_rate_hz": 2.0,  # 突き合わせと「何が見えていたか」にはこれで足りる
        }],
    )
