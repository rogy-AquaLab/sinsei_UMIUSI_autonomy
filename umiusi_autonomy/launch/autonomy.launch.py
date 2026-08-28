"""Launch the UMIUSI deploy autonomy pipeline: camera_bridge_node + perception_node + navigator_node.

    ros2 launch umiusi_autonomy autonomy.launch.py

This is the **standalone (no-core)** path: navigator_node drives ``/cmd/direct`` itself. To ride on
core's power/mode pipeline instead, use ``core_autonomy.launch.py``.

実機カメラ (gst_camera_node) は RTSP に流すだけで ROS トピックを出さないので、``camera_bridge_node``
が要る。**以前ここにブリッジが無く、実機では画像が 1 枚も来ずに perception が沈黙し、FSM が
SEARCH から出られなかった** (8/25 の水中 run、``/perception_node/detections`` が 15.6 分間ゼロ)。
sim やオフラインの image publisher を使うときは ``use_camera_bridge:=false``。

Nodes load their parameters from config/autonomy.yaml; the common overrides are exposed as launch
arguments. Use ``publish:=false`` to run the FSM without commanding the thrusters.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from umiusi_autonomy.launch_common import camera_bridge_node


def generate_launch_description():
    params = PathJoinSubstitution([FindPackageShare("umiusi_autonomy"), "config", "autonomy.yaml"])
    model_path = LaunchConfiguration("model_path")
    image_topic = LaunchConfiguration("image_topic")
    publish = LaunchConfiguration("publish")
    max_duty = LaunchConfiguration("max_duty")
    use_camera_bridge = LaunchConfiguration("use_camera_bridge")
    rtsp_url = LaunchConfiguration("rtsp_url")
    record_vision = LaunchConfiguration("record_vision")

    return LaunchDescription([
        DeclareLaunchArgument("model_path", default_value="",
                              description="learned detector .pt checkpoint。"
                                          "空なら同梱の camp_real.pt (models/detector/README.md)"),
        DeclareLaunchArgument("image_topic", default_value="/front_cam/image_raw",
                              description="onboard camera topic"),
        DeclareLaunchArgument("publish", default_value="true",
                              description="navigator commands the thrusters (false = dry / FSM-only)"),
        DeclareLaunchArgument("max_duty", default_value="0.25",
                              description="duty upper bound (this path bypasses control's max_duty)"),
        DeclareLaunchArgument("use_camera_bridge", default_value="true",
                              description="RTSP -> ROS Image のブリッジを起動する。実機カメラは "
                                          "gst_camera_node が RTSP に流すだけで ROS トピックを "
                                          "出さないため、perception にはこれが必要。sim や "
                                          "別の image publisher を使うときだけ false"),
        DeclareLaunchArgument("rtsp_url", default_value="rtsp://localhost:8554/cam1",
                              description="ブリッジが読む RTSP URL (前方カメラ = cam1)"),
        DeclareLaunchArgument("record_vision", default_value="false",
                              description="圧縮画像 (<image_topic>/compressed) も出す。"
                                          "`record_run.sh --vision` で bag に残すため — "
                                          "視覚での位置固定を作る素材集め用 (docs/logging.md)"),

        # --- 実機カメラ映像を perception に渡す (core_autonomy.launch.py と同じ設定) ---
        camera_bridge_node(condition=use_camera_bridge, rtsp_url=rtsp_url,
                           image_topic=image_topic, record_vision=record_vision),
        Node(
            package="umiusi_autonomy",
            executable="perception_node",
            name="perception_node",
            output="screen",
            parameters=[params, {"model_path": model_path, "image_topic": image_topic}],
            # 実機では torch のスレッドを 1 に固定する (core_autonomy.launch.py と同じ理由)。
            additional_env={"OMP_NUM_THREADS": "1", "MKL_NUM_THREADS": "1"},
        ),
        Node(
            package="umiusi_autonomy",
            executable="navigator_node",
            name="navigator_node",
            output="screen",
            parameters=[params, {"publish": publish, "max_duty": max_duty}],
        ),
    ])
