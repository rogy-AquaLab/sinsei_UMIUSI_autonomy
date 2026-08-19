"""Launch the UMIUSI deploy autonomy pipeline: perception_node + navigator_node.

    ros2 launch umiusi_autonomy autonomy.launch.py model_path:=/abs/path/to/detector.pt

Both nodes load their parameters from config/autonomy.yaml; ``model_path`` (the learned detector
checkpoint, required by perception_node) and ``image_topic`` are also exposed as launch arguments
for the common overrides. Use ``publish:=false`` to run the FSM without commanding the thrusters.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = PathJoinSubstitution([FindPackageShare("umiusi_autonomy"), "config", "autonomy.yaml"])
    model_path = LaunchConfiguration("model_path")
    image_topic = LaunchConfiguration("image_topic")
    publish = LaunchConfiguration("publish")

    return LaunchDescription([
        DeclareLaunchArgument("model_path", default_value="",
                              description="learned detector .pt checkpoint (REQUIRED)"),
        DeclareLaunchArgument("image_topic", default_value="/front_cam/image_raw",
                              description="onboard camera topic"),
        DeclareLaunchArgument("publish", default_value="true",
                              description="navigator commands the thrusters (false = dry / FSM-only)"),
        Node(
            package="umiusi_autonomy",
            executable="perception_node",
            name="perception_node",
            output="screen",
            parameters=[params, {"model_path": model_path, "image_topic": image_topic}],
        ),
        Node(
            package="umiusi_autonomy",
            executable="navigator_node",
            name="navigator_node",
            output="screen",
            parameters=[params, {"publish": publish}],
        ),
    ])
