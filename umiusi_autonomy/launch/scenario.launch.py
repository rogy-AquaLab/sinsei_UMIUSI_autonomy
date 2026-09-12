"""競技シナリオを姿勢制御の上で回す — 認識 + FSM + 姿勢制御器。

    ros2 launch umiusi_autonomy scenario.launch.py                    # 一式
    ros2 launch umiusi_autonomy scenario.launch.py publish:=false     # 指令を出さず計算だけ
    ros2 launch umiusi_autonomy scenario.launch.py max_duty:=0.4
    ros2 launch umiusi_autonomy scenario.launch.py use_attitude:=false  # 姿勢制御器は別に上げる
    ros2 launch umiusi_autonomy scenario.launch.py use_perception:=false # FSM だけ (teleop 併用)

**control は別に上げること** (`umiusi_stack.sh start --control-only` または
`bringup.launch.py mode:=scenario`)。こちらは control を面倒みない — 段階起動と
IMU 待ちが要るなら `bringup.launch.py mode:=scenario` を使う。**このファイルは
「control は既に居る」前提で、シナリオの 3 ノードだけを出し入れするためのもの。**

立ち上がるもの:
  * `perception_node`   検出器 (既定は同梱の camp_real2.pt)
  * `navigator_node`    FSM。`command_mode=setpoint` で AttitudeTarget を出すだけ
  * `classical_attitude` 姿勢の安定化 + 4 基への配分。**スラスタを叩くのはここだけ**

起動しただけでは動かない (DISARMED)。arm はサービスで:
    ros2 service call /classical_attitude/arm std_srvs/srv/SetBool '{data: true}'

手順と「今どこまで動くか」は docs/scenario_run.md。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    publish = LaunchConfiguration("publish")
    max_duty = LaunchConfiguration("max_duty")

    def _share(name):
        return PathJoinSubstitution([FindPackageShare("umiusi_autonomy"), "launch", name])

    # 認識 + FSM。command_mode=setpoint なので navigator はスラスタを叩かない
    fsm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_share("autonomy.launch.py")),
        launch_arguments={
            "publish": publish,
            "command_mode": "setpoint",
            "setpoint_topic": LaunchConfiguration("setpoint_topic"),
            "model_path": LaunchConfiguration("model_path"),
            "rtsp_url": LaunchConfiguration("rtsp_url"),
            "use_camera_bridge": LaunchConfiguration("use_camera_bridge"),
        }.items(),
        condition=IfCondition(LaunchConfiguration("use_perception")))

    # 姿勢制御器。**disarmed で上がる** — 起動しただけで推力が出るのを避ける
    attitude = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(_share("classical_attitude.launch.py")),
        launch_arguments={"publish": publish, "max_duty": max_duty,
                          "hold_yaw": LaunchConfiguration("hold_yaw"),
                          "k_v_vert": LaunchConfiguration("k_v_vert"),
                          "cmd_target_topic": LaunchConfiguration("cmd_target_topic")}.items(),
        condition=IfCondition(LaunchConfiguration("use_attitude")))

    return LaunchDescription([
        DeclareLaunchArgument("publish", default_value="true",
                              description="false でスラスタへ出さず計算だけ (ドライ確認)"),
        DeclareLaunchArgument("max_duty", default_value="0.25",
                              description="姿勢制御器の duty 上限。"
                                          "cap 0.25 は姿勢誤差 10 度で飽和する (実測)"),
        DeclareLaunchArgument("hold_yaw", default_value="true",
                              description="false で yaw の保持だけ切る"),
        DeclareLaunchArgument("k_v_vert", default_value="-1.0",
                              description="鉛直の速度フィードバック。負でバンドルの値 "
                                          "(既定 0 = 前進項だけ)"),
        DeclareLaunchArgument("cmd_target_topic", default_value="",
                              description="空以外にすると `/cmd/target` も目標として受ける "
                                          "(UI のテレオペを姿勢制御の上に乗せる)"),
        DeclareLaunchArgument("setpoint_topic",
                              default_value="/classical_attitude/setpoint"),
        DeclareLaunchArgument("model_path", default_value="",
                              description="検出器の .pt。空なら同梱のもの"),
        DeclareLaunchArgument("rtsp_url", default_value="rtsp://127.0.0.1:8554/cam1"),
        DeclareLaunchArgument("use_camera_bridge", default_value="true"),
        DeclareLaunchArgument("use_perception", default_value="true",
                              description="false で認識と FSM を上げない "
                                          "(姿勢制御器だけ = teleop 用)"),
        DeclareLaunchArgument("use_attitude", default_value="true",
                              description="false で姿勢制御器を上げない "
                                          "(RL を代わりに使うときなど)"),
        attitude,
        fsm,
    ])
