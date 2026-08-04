"""Start ONLY the RL attitude(-velocity) controller — the trained policy driving the thrusters.

    ros2 launch umiusi_autonomy rl_attitude.launch.py                    # bundled cruise policy
    ros2 launch umiusi_autonomy rl_attitude.launch.py vel_cmd:=0.3       # slower cruise
    ros2 launch umiusi_autonomy rl_attitude.launch.py publish:=false     # predict only (no thrusters)
    ros2 launch umiusi_autonomy rl_attitude.launch.py model_path:=/abs/final.zip

Runs just ``rl_attitude_node`` (no perception / FSM / core). It holds upright + cruises body +X using
the bundled ``models/cruise_policy`` policy. Needs stable-baselines3 + torch + gymnasium in the ROS
runtime env, and the controllers/bridge (sinsei_umiusi_control or umiusi_sim_bridge) providing
``/state/imu_state`` + ``/state/thruster_state_all`` and consuming ``/cmd/direct/...``.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    model_path = LaunchConfiguration("model_path")
    vel_cmd = LaunchConfiguration("vel_cmd")
    publish = LaunchConfiguration("publish")

    return LaunchDescription([
        DeclareLaunchArgument("model_path", default_value="",
                              description="policy .zip (empty = bundled models/cruise_policy/final.zip)"),
        DeclareLaunchArgument("vel_cmd", default_value="0.4",
                              description="forward (+X) commanded speed [m/s]"),
        DeclareLaunchArgument("publish", default_value="true",
                              description="command the thrusters (false = predict only)"),
        Node(
            package="umiusi_autonomy",
            executable="rl_attitude_node",
            name="rl_attitude_node",
            output="screen",
            # model_path="" -> the node falls back to the bundled models/cruise_policy/final.zip
            parameters=[{"model_path": model_path, "vel_cmd": vel_cmd, "publish": publish}],
        ),
    ])
