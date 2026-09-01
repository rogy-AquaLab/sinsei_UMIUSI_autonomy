"""実機スタックを 1 本の launch で段階起動する。tools/umiusi_stack.sh の launch 版。

    ros2 launch umiusi_autonomy stack.launch.py
    ros2 launch umiusi_autonomy stack.launch.py mode:=attitude   # control + RL (カメラなし)
    ros2 launch umiusi_autonomy stack.launch.py mode:=perception # control + 認識のみ

段を分けるのは依存関係ではなく起動時の CPU 競合のため — Pi では torch を 2 回読む間に
controller_manager と xacro が走る。待つのは固定秒ではなく実際の完了シグナル:

  * control -> /state/imu の最初のメッセージ (controller_manager の spawner が終わると出る)
  * 認識    -> /perception_node/detections の最初のメッセージ (検出器のロード + 初フレーム)

シグナルが来なくても timeout で先へ進む (wait_for_topic の --allow-timeout)。進んでしまう
のが困る状況では、そのトピックが本当に出ていないということなのでログを見ること。

IMU の待ちは use_control に関係なく行う。「IMU が流れていること」が本当の前提条件で、
それを control が出すか sim bridge が出すかは問わない。

段の順序 (mode ごと):
  full       control -> 認識 -> RL
  perception control -> 認識
  attitude   control -> RL        (認識を上げないので RL は IMU の直後)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

# 段の定義。timeout は umiusi_stack.sh の固定 sleep と同じ値 — シグナルで抜けるので通常は
# ここまで待たないが、上限として越えないことを保証する。
# best_effort: センサ系の publisher は BEST_EFFORT が普通で、RELIABLE では繋がらない。
# modes: その段を待つ mode。空 = 常に待つ。
STAGES = (
    {"name": "wait_control", "topic": "/state/imu", "timeout": "20",
     "best_effort": True, "modes": ()},
    {"name": "wait_perception", "topic": "/perception_node/detections", "timeout": "35",
     "best_effort": False, "modes": ("full",)},
)


def wait_args(stage: dict) -> list:
    """段の定義 -> wait_for_topic の引数。--allow-timeout は必須 (段を止めないため)。"""
    args = ["--topic", stage["topic"], "--timeout", stage["timeout"], "--allow-timeout"]
    if stage["best_effort"]:
        args.append("--best-effort")
    return args


def _wait(stage: dict, mode) -> Node:
    return Node(package="umiusi_autonomy", executable="wait_for_topic", name=stage["name"],
                arguments=wait_args(stage), output="screen",
                condition=_mode_is(mode, *stage["modes"]) if stage["modes"] else None)


def _mode_is(mode, *names):
    return IfCondition(PythonExpression(["'", mode, "' in ", repr(names)]))


def generate_launch_description():
    mode = LaunchConfiguration("mode")
    use_control = LaunchConfiguration("use_control")
    model_path = LaunchConfiguration("model_path")
    rtsp_url = LaunchConfiguration("rtsp_url")
    publish = LaunchConfiguration("publish")

    control = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("sinsei_umiusi_control"), "launch", "main.yaml"])),
        condition=IfCondition(use_control))

    autonomy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("umiusi_autonomy"), "launch", "core_autonomy.launch.py"])),
        launch_arguments={"model_path": model_path, "rtsp_url": rtsp_url,
                          "use_camera_bridge": "true"}.items(),
        condition=_mode_is(mode, "full", "perception"))

    def _rl(condition):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution(
                [FindPackageShare("umiusi_rl_control"), "launch", "rl_attitude.launch.py"])),
            launch_arguments={"publish": publish}.items(), condition=condition)

    # RL の起動点は mode で変わる。認識を上げるなら認識のロードが終わってから、
    # 上げないなら IMU の直後。両方に同じ action を渡すと二重起動になるので分ける
    rl_after_percep = _rl(_mode_is(mode, "full"))
    rl_after_control = _rl(_mode_is(mode, "attitude"))

    wait_control = _wait(STAGES[0], mode)
    wait_percep = _wait(STAGES[1], mode)

    return LaunchDescription([
        DeclareLaunchArgument("mode", default_value="full",
                              choices=["full", "attitude", "perception"],
                              description="full = control + 認識 + RL / attitude = control + RL / "
                                          "perception = control + 認識"),
        DeclareLaunchArgument("use_control", default_value="true",
                              description="false で sinsei_umiusi_control を起動しない "
                                          "(sim bridge を自分で立てているとき)。IMU の待ちは残る"),
        DeclareLaunchArgument("model_path", default_value="",
                              description="検出器の .pt。空なら同梱のもの"),
        DeclareLaunchArgument("rtsp_url", default_value="rtsp://127.0.0.1:8554/cam1",
                              description="カメラブリッジの入力"),
        DeclareLaunchArgument("publish", default_value="true",
                              description="false で RL の指令をスラスタへ出さない (ドライ試験)"),

        control,
        wait_control,
        RegisterEventHandler(OnProcessExit(
            target_action=wait_control, on_exit=[autonomy, wait_percep, rl_after_control])),
        RegisterEventHandler(OnProcessExit(
            target_action=wait_percep, on_exit=[rl_after_percep])),
    ])
