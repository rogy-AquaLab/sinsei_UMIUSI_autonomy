"""Start ONLY the RL attitude(-velocity) controller — the trained policy driving the thrusters.

    ros2 launch umiusi_rl_control rl_attitude.launch.py                    # 姿勢保持のみ (disarmed で起動)
    ros2 launch umiusi_rl_control rl_attitude.launch.py vel_cmd:=0.4       # 巡航 (前進 0.4 m/s)
    ros2 launch umiusi_rl_control rl_attitude.launch.py start_armed:=true  # 起動と同時に武装
    ros2 launch umiusi_rl_control rl_attitude.launch.py publish:=false     # predict only (no thrusters)
    ros2 launch umiusi_rl_control rl_attitude.launch.py model_path:=/abs/final.zip

Runs just ``rl_attitude_node`` (no perception / FSM / core). It holds upright using the bundled
``models/cruise_policy`` policy.

**既定は disarmed + vel_cmd 0** なので、起動しただけではスラスタに何も出ない。
``~/arm`` (std_srvs/SetBool, data:true) で武装し、必要なら ``vel_cmd`` で前進させる。

Needs stable-baselines3 + torch + gymnasium in the ROS runtime env, and the controllers/bridge
(sinsei_umiusi_control or umiusi_sim_bridge) providing ``/state/imu`` +
``/state/thruster_state_all`` and consuming ``/cmd/direct/...``.
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

    return LaunchDescription([
        DeclareLaunchArgument("model_path", default_value="",
                              description="policy .zip (empty = bundled models/cruise_policy/final.zip)"),
        DeclareLaunchArgument("vel_cmd", default_value="0.0",
                              description="forward (+X) commanded speed [m/s]。**既定 0** — "
                                          "起動しただけで前進指令が出るのを避けるため。巡航は 0.4"),
        DeclareLaunchArgument("publish", default_value="true",
                              description="command the thrusters (false = predict only)"),
        DeclareLaunchArgument("start_armed", default_value="false",
                              description="起動と同時に武装する。**既定 false** — 起動しただけで "
                                          "スラスタへ指令が出るのを避けるため。`~/arm` で武装する"),
        Node(
            package="umiusi_rl_control",
            executable="rl_attitude_node",
            name="rl_attitude_node",
            output="screen",
            # model_path="" -> the node falls back to the bundled models/cruise_policy/final.zip
            parameters=[{"model_path": model_path, "vel_cmd": vel_cmd, "publish": publish,
                         "start_armed": start_armed}],
        ),
    ])
