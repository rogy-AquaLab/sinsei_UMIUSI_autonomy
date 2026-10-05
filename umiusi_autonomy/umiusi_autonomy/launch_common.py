"""launch ファイルの共通部品。

autonomy.launch.py と core_autonomy.launch.py はどちらもカメラブリッジを起動する。
同じ Node ブロックを 2 箇所に書くと 片方だけ直る (IMU の扱いを ImuSource に寄せたのと
同じ構図)。launch ディレクトリは Python パッケージではないので、共有するものは
インストールされるこのパッケージ側に置く。
"""

from __future__ import annotations

from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# 規約: sinsei_umiusi_control の params/cameras_shm.yaml (pi_camera の shmsink) と同じパス
SHM_SOCKET = "/tmp/umiusi_cam1.sock"


def camera_source_arg() -> DeclareLaunchArgument:
    return DeclareLaunchArgument(
        "camera_source", default_value="rtsp", choices=["rtsp", "shm"],
        description="カメラブリッジの入力。shm は control を cameras_shm.yaml で起動したときだけ "
                    "(エンコード前の映像を共有メモリから読み、H.264 のデコードを省く)")


def shm_socket_for(camera_source) -> PythonExpression:
    """camera_source:=shm のときだけ SHM_SOCKET、それ以外は空 (= RTSP)。"""
    return PythonExpression([f"'{SHM_SOCKET}' if '", camera_source, "' == 'shm' else ''"])


def camera_bridge_node(*, condition, rtsp_url, image_topic, record_vision="false", max_fps="15") -> Node:
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
            # camera_source を宣言していない launch から呼ばれたら RTSP
            "shm_socket": ParameterValue(
                shm_socket_for(LaunchConfiguration("camera_source", default="rtsp")),
                value_type=str),
            "image_topic": image_topic,
            "width": 320, "height": 240,   # autonomy.yaml の frame_w/frame_h に合わせる
            "max_rate_hz": 0.0,            # 制限をかけると取りこぼす (実測) — 絞るなら max_fps で
            "max_fps": ParameterValue(max_fps, value_type=int),
            "auto_rate": False,            # AIMD 追従は実験的。既定は無効
            "publish_compressed": record_vision,
            "compressed_max_rate_hz": 2.0,  # 突き合わせと「何が見えていたか」にはこれで足りる
        }],
    )
