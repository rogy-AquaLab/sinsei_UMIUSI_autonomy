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

    return LaunchDescription([
        DeclareLaunchArgument("model_path", default_value="",
                              description="learned detector .pt checkpoint (REQUIRED for perception)"),
        DeclareLaunchArgument("image_topic", default_value="/front_cam/image_raw",
                              description="onboard camera topic"),
        DeclareLaunchArgument("use_rosbridge", default_value="true",
                              description="also start rosbridge_websocket (as core's main.yaml does)"),

        # --- autonomy side (umiusi_autonomy) ---
        Node(
            package="umiusi_autonomy",
            executable="perception_node",
            name="perception_node",
            output="screen",
            parameters=[params, {"model_path": model_path, "image_topic": image_topic}],
        ),
        Node(
            package="umiusi_autonomy",
            executable="auto_target_generator",   # FSM-driven Target; replaces core's placeholder
            name="auto_target_generator",
            output="screen",
            parameters=[params],
        ),

        # --- core strategy stack (sinsei_umiusi_core), minus its auto_target_generator ---
        Node(package="sinsei_umiusi_core", executable="low_power_health_check", output="screen"),
        Node(package="sinsei_umiusi_core", executable="manual_target_generator", output="screen"),
        Node(
            package="sinsei_umiusi_core",
            executable="robot_strategy",
            namespace="robot_strategy",
            output="screen",
            parameters=[{"behavior_tree_file": behavior_tree_file}],
        ),
        Node(
            package="rosbridge_server",
            executable="rosbridge_websocket",
            output="screen",
            condition=IfCondition(use_rosbridge),
        ),
    ])
