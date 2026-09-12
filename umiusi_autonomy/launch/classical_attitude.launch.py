"""古典制御の姿勢制御だけを上げる。RL の rl_attitude.launch.py と対になるもの。

    ros2 launch umiusi_autonomy classical_attitude.launch.py                  # 指令を出す
    ros2 launch umiusi_autonomy classical_attitude.launch.py publish:=false   # 計算だけ
    ros2 launch umiusi_autonomy classical_attitude.launch.py max_duty:=0.3

**rl_attitude.launch.py と同時に起動しないこと。** 同じ /cmd/direct を取り合う。
control 側は別に上げておく (docs/field_card.md)。

bundle は同梱のものを既定で使う。較正したら umiusi_sim の tools/export_classical.py を
流し直して config/classical_bundle.json を置き換えること — config だけ直しても
このファイルは古いまま残る (known_issues A-11 と同じ罠)。
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    args = [
        # 空 = 同梱の config/classical_bundle.json
        DeclareLaunchArgument("bundle_path", default_value=""),
        DeclareLaunchArgument("max_duty", default_value="0.25",
                              description="duty の絶対値上限。力は上限の 2 乗で効く"),
        DeclareLaunchArgument("publish", default_value="true",
                              description="false でスラスタへ出さず計算だけ (ドライ確認)"),
        DeclareLaunchArgument("start_armed", default_value="false"),
        DeclareLaunchArgument("vel_cmd", default_value="0.0",
                              description="前進 (+X) の速度指令 [m/s]"),
        DeclareLaunchArgument("imu_sanity_enforce", default_value="false",
                              description="true で化けサンプルを破棄する。既定は検出のみ"),
        # 現場でコードを触らずに符号を直すための入口。実行中も `ros2 param set` で変えられる
        DeclareLaunchArgument("thrust_sign", default_value="[1.0, 1.0, 1.0, 1.0]",
                              description="基ごとの推力の向き (lf, lb, rb, rf)。-1.0 で反転。"
                                          "値は tools/thrust_sign_check.py が測って出す"),
        DeclareLaunchArgument("servo_sign", default_value="[1.0, 1.0, 1.0, 1.0]",
                              description="基ごとのサーボ回転センス (lf, lb, rb, rf)"),
        DeclareLaunchArgument("hold_yaw", default_value="true",
                              description="false で yaw の保持だけ切る (roll/pitch のみ保つ)"),
        DeclareLaunchArgument("k_v_vert", default_value="-1.0",
                              description="鉛直 (heave) の速度フィードバック。"
                                          "負でバンドルの値 (既定 0 = 前進項だけ)"),
        DeclareLaunchArgument("cmd_target_topic", default_value="",
                              description="空以外で `/cmd/target` も目標として受ける。"
                                          "UI のテレオペが姿勢制御の上に乗る"),
    ]
    return LaunchDescription(args + [
        Node(
            package="umiusi_autonomy",
            executable="classical_attitude_node",
            name="classical_attitude",
            output="screen",
            parameters=[{
                "bundle_path": LaunchConfiguration("bundle_path"),
                "max_duty": LaunchConfiguration("max_duty"),
                "publish": LaunchConfiguration("publish"),
                "start_armed": LaunchConfiguration("start_armed"),
                "vel_cmd": LaunchConfiguration("vel_cmd"),
                "imu_sanity_enforce": LaunchConfiguration("imu_sanity_enforce"),
                "thrust_sign": ParameterValue(LaunchConfiguration("thrust_sign"),
                                              value_type=None),
                "servo_sign": ParameterValue(LaunchConfiguration("servo_sign"),
                                             value_type=None),
                "hold_yaw": ParameterValue(LaunchConfiguration("hold_yaw"), value_type=bool),
                "k_v_vert": ParameterValue(LaunchConfiguration("k_v_vert"), value_type=float),
                "cmd_target_topic": LaunchConfiguration("cmd_target_topic"),
            }],
        ),
    ])
