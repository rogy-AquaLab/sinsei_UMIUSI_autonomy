"""Bring up the UMIUSI strategy stack with autonomy RIDING ON CORE — without modifying core.

    ros2 launch umiusi_autonomy core_autonomy.launch.py model_path:=/abs/path/to/detector.pt

This mirrors ``sinsei_umiusi_core``'s ``launch/main.yaml`` node-for-node, EXCEPT the AUTO-mode target
source: instead of core's empty-Target placeholder ``auto_target_generator`` it starts umiusi_autonomy's
FSM-driven lifecycle ``auto_target_generator`` (same node name + lifecycle contract, so core's behaviour
tree activates it on entering AUTO). It also starts ``perception_node`` to feed the generator. Because
this launch owns the whole strategy stack, ``sinsei_umiusi_core`` is left untouched — do NOT also run
core's ``main.yaml`` (that would start a second generator racing on ``/cmd/target``).

The controllers + hardware (sinsei_umiusi_control / the sim bridge) are launched separately, as usual.
EXPERIMENTAL: validate on sim/hardware before use — see the ``auto_target_generator`` module docstring.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = PathJoinSubstitution([FindPackageShare("umiusi_autonomy"), "config", "autonomy.yaml"])
    behavior_tree_file = PathJoinSubstitution(
        [FindPackageShare("sinsei_umiusi_core"), "behavior_tree", "tree_main.xml"])
    model_path = LaunchConfiguration("model_path")
    image_topic = LaunchConfiguration("image_topic")
    use_rosbridge = LaunchConfiguration("use_rosbridge")
    use_camera_bridge = LaunchConfiguration("use_camera_bridge")
    use_core = LaunchConfiguration("use_core")
    rtsp_url = LaunchConfiguration("rtsp_url")

    return LaunchDescription([
        DeclareLaunchArgument("model_path", default_value="",
                              description="learned detector .pt checkpoint。空なら同梱の camp_mix.pt を使う。実際の水中は camp_real.pt のほうが強い (models/detector/README.md)"),
        DeclareLaunchArgument("image_topic", default_value="/front_cam/image_raw",
                              description="onboard camera topic"),
        DeclareLaunchArgument("use_rosbridge", default_value="true",
                              description="also start rosbridge_websocket (as core's main.yaml does). "
                                          "UI を使わない運用では false にすると CPU が空く"),
        DeclareLaunchArgument("use_camera_bridge", default_value="true",
                              description="RTSP -> ROS Image のブリッジを起動する。実機カメラは "
                                          "gst_camera_node が RTSP に流すだけで ROS トピックを "
                                          "出さないため、perception にはこれが必要"),
        DeclareLaunchArgument("use_core", default_value="true",
                              description="core の BT スタック (robot_strategy / manual_target_generator "
                                          "/ low_power_health_check) も起動する。**perception を単体で"
                                          "見るときは false** — BT が無いぶん CPU が空き、"
                                          "`/cmd/target` も出ない (auto_target_generator は "
                                          "unconfigured のまま)"),
        DeclareLaunchArgument("rtsp_url", default_value="rtsp://localhost:8554/cam1",
                              description="ブリッジが読む RTSP URL (前方カメラ = cam1)"),

        # --- 実機カメラ映像を perception に渡す (umiusi_autonomy) ---
        Node(
            package="umiusi_autonomy",
            executable="camera_bridge_node",
            name="camera_bridge_node",
            output="screen",
            condition=IfCondition(use_camera_bridge),
            parameters=[{
                "rtsp_url": rtsp_url,
                "image_topic": image_topic,
                "width": 320, "height": 240,   # autonomy.yaml の frame_w/frame_h に合わせる
                "max_rate_hz": 0.0,            # 制限をかけると取りこぼす (実測) — カメラ側で絞ること
                "auto_rate": False,            # AIMD 追従は実験的。既定は無効
            }],
        ),

        # --- autonomy side (umiusi_autonomy) ---
        Node(
            package="umiusi_autonomy",
            executable="perception_node",
            name="perception_node",
            output="screen",
            parameters=[params, {"model_path": model_path, "image_topic": image_topic}],
            # 実機では torch のスレッドを 1 に固定する。他ノードと CPU を奪い合うと
            # スレッドを増やすほど遅くなる (実測: 負荷下で 4 スレッド 142 ms/frame に対し
            # 1 スレッド 113.9 ms/frame。エンドツーエンドでも 5.32 -> 6.34 Hz)。
            additional_env={"OMP_NUM_THREADS": "1", "MKL_NUM_THREADS": "1"},
        ),
        Node(
            package="umiusi_autonomy",
            executable="auto_target_generator",   # FSM-driven Target; replaces core's placeholder
            name="auto_target_generator",
            output="screen",
            parameters=[params],
        ),

        # --- core strategy stack (sinsei_umiusi_core), minus its auto_target_generator ---
        Node(package="sinsei_umiusi_core", executable="low_power_health_check", output="screen",
             condition=IfCondition(use_core)),
        Node(package="sinsei_umiusi_core", executable="manual_target_generator", output="screen",
             condition=IfCondition(use_core)),
        Node(
            package="sinsei_umiusi_core",
            executable="robot_strategy",
            namespace="robot_strategy",
            output="screen",
            condition=IfCondition(use_core),
            parameters=[{"behavior_tree_file": behavior_tree_file}],
        ),
        Node(
            package="rosbridge_server",
            executable="rosbridge_websocket",
            output="screen",
            condition=IfCondition(use_rosbridge),
        ),
    ])
