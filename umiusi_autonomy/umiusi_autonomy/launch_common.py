"""launch ファイルの共通部品。

`autonomy.launch.py` と `core_autonomy.launch.py` はどちらもカメラブリッジを起動する。
同じ Node ブロックを 2 箇所に書くと **片方だけ直る** (IMU の扱いを `ImuSource` に寄せたのと
同じ構図)。launch ディレクトリは Python パッケージではないので、共有するものは
インストールされるこのパッケージ側に置く。
"""

from __future__ import annotations

from launch.conditions import IfCondition
from launch_ros.actions import Node


def camera_bridge_node(*, condition, rtsp_url, image_topic) -> Node:
    """RTSP -> ROS Image のブリッジ。

    実機カメラ (gst_camera_node) は RTSP に流すだけで ROS トピックを出さないので、
    perception にはこれが必要。これが無いと画像が 1 枚も来ず、FSM が SEARCH から出られない
    (8/25 の水中 run、`/perception_node/detections` が 15.6 分間ゼロ)。
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
        }],
    )
