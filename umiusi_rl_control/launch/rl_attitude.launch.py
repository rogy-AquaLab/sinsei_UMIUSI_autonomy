"""Start ONLY the RL attitude(-velocity) controller — the trained policy driving the thrusters.

    ros2 launch umiusi_rl_control rl_attitude.launch.py                    # 姿勢保持のみ (disarmed で起動)
    ros2 launch umiusi_rl_control rl_attitude.launch.py vel_cmd:=0.4       # 巡航 (前進 0.4 m/s)
    ros2 launch umiusi_rl_control rl_attitude.launch.py start_armed:=true  # 起動と同時に武装
    ros2 launch umiusi_rl_control rl_attitude.launch.py hold_yaw:=false    # roll/pitch だけ保つ
    ros2 launch umiusi_rl_control rl_attitude.launch.py max_duty:=0.4      # 出力上限を上げる

    # 姿勢保持専用ポリシー (14 次元、フォールバック) で
    ros2 launch umiusi_rl_control rl_attitude.launch.py \
        model_path:=$(ros2 pkg prefix umiusi_rl_control)/share/umiusi_rl_control/models/att_cal1_best_rep103
    ros2 launch umiusi_rl_control rl_attitude.launch.py publish:=false     # predict only (no thrusters)

Runs just rl_attitude_node (no perception / FSM / core). 既定は同梱の
models/av_cal1_best_rep103 (本命、姿勢+速度指令 17 次元)。読み込み時に frame 契約
(rep103) と golden.npz を検証し、不一致なら動かさない。

既定は disarmed + vel_cmd 0 なので、起動しただけではスラスタに何も出ない。
~/arm (std_srvs/SetBool, data:true) で武装し、必要なら vel_cmd で前進させる。

Needs torch + numpy in the ROS runtime env (SB3 は不要), and the controllers/bridge
(sinsei_umiusi_control or umiusi_sim_bridge) providing /state/imu and consuming
/cmd/direct/....
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    model_path = LaunchConfiguration("model_path")
    vel_cmd = LaunchConfiguration("vel_cmd")
    publish = LaunchConfiguration("publish")
    start_armed = LaunchConfiguration("start_armed")
    hold_yaw = LaunchConfiguration("hold_yaw")
    max_duty = LaunchConfiguration("max_duty")
    depth_supervisor = LaunchConfiguration("depth_supervisor")
    target_depth = LaunchConfiguration("target_depth")
    vel_timeout = LaunchConfiguration("vel_timeout")

    return LaunchDescription([
        DeclareLaunchArgument("model_path", default_value="",
                              description="policy bundle dir (empty = bundled models/av_cal1_best_rep103)"),
        DeclareLaunchArgument("vel_cmd", default_value="0.0",
                              description="forward (+X) commanded speed [m/s]。新ポリシーは停止保持 (0) も"
                                          "学習分布内。巡航試験では 0.4 に上げる"),
        DeclareLaunchArgument("publish", default_value="true",
                              description="command the thrusters (false = predict only)"),
        DeclareLaunchArgument("hold_yaw", default_value="true",
                              description="yaw も保持する。false で roll/pitch だけ保つ "
                                          "(手で回したときに戻そうとして回り続けるのを避ける)。"
                                          "実行中も `ros2 param set` で切り替えられる"),
        DeclareLaunchArgument("max_duty", default_value="0.25",
                              description="duty_cycle の絶対値上限 (1.0 = 制限なし)。**0.25 で開始**。"
                                          "0.2 は 96% 飽和で比例制御にならず降下もできない "
                                          "(8/25 の水中 run)。**0.4 は零空間を潰してから** — 上限は"
                                          "力の次元で効くので 0.2→0.4 は 4 倍 (issue #19)"),
        DeclareLaunchArgument("vel_timeout", default_value="0.0",
                              description="デッドマン: 速度指令がこの秒数更新されなければ 0 に戻す "
                                          "(0 以下で無効)。狭いプールの巡航試験では 5.0 を推奨"),
        DeclareLaunchArgument("depth_supervisor", default_value="false",
                              description="深度モード切替を有効化 (水圧センサ搭載時のみ)。"
                                          "**max_duty 0.3 以上を推奨** — 0.2 で降下できないのは "
                                          "上限ではなく零空間への配分が原因 (issue #19)。"
                                          "詳細は depth_supervisor.py 冒頭"),
        DeclareLaunchArgument("target_depth", default_value="0.0",
                              description="目標深度 [m, 正=深い]。実行中に "
                                          "`ros2 param set /rl_attitude_node target_depth 1.0` で変更"),
        DeclareLaunchArgument("start_armed", default_value="false",
                              description="起動と同時に武装する。**既定 false** — 起動しただけで "
                                          "スラスタへ指令が出るのを避けるため。`~/arm` で武装する"),
        Node(
            package="umiusi_rl_control",
            executable="rl_attitude_node",
            name="rl_attitude_node",
            output="screen",
            # model_path="" -> the node falls back to the bundled models/av_cal1_best_rep103
            parameters=[{"model_path": model_path, "vel_cmd": vel_cmd, "publish": publish,
                         "start_armed": start_armed, "hold_yaw": hold_yaw,
                         "max_duty": max_duty, "depth_supervisor": depth_supervisor,
                         "target_depth": target_depth, "vel_timeout": vel_timeout}],
        ),
    ])
