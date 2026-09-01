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
# best_effort: 誰が publish するか分からない段では緩い側にする (/state/imu は control か
# sim bridge)。BEST_EFFORT の購読はどちらの publisher にも繋がる。
# modes: その段を待つ mode。空 = 常に待つ。
# 認識を上げる mode。段の待ちと include の条件を同じ定数から引く — 別々に書くと
# 「認識は上がるのに待たない」mode ができる (実際 perception でそうなっていた)
PERCEPTION_MODES = ("full", "perception")

# トピック名は perception_node の detections_topic (config/autonomy.yaml) と揃えること。
# core_autonomy.launch.py がその yaml を固定で渡すので、いまはずれようがない
STAGES = (
    {"name": "wait_control", "topic": "/state/imu", "timeout": "20",
     "best_effort": True, "modes": ()},
    {"name": "wait_perception", "topic": "/perception_node/detections", "timeout": "35",
     "best_effort": False, "modes": PERCEPTION_MODES},
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
    use_ui = LaunchConfiguration("use_ui")

    # カメラ設定は既定で同梱の cameras_deploy.yaml を渡す。渡さないと実機既定の
    # /dev/video2 (H264 非対応) が使われてカメラが開かない (known_issues B-1)
    cameras_param_file = PythonExpression([
        "'", LaunchConfiguration("cameras_param_file"), "' or '",
        PathJoinSubstitution([FindPackageShare("umiusi_autonomy"), "config",
                              "cameras_deploy.yaml"]), "'"])
    control = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("sinsei_umiusi_control"), "launch", "main.yaml"])),
        launch_arguments={
            # attitude はカメラを上げない (CPU を空ける)
            "enable_cameras": PythonExpression(
                ["'false' if '", mode, "' == 'attitude' else 'true'"]),
            "cameras_param_file": cameras_param_file,
        }.items(),
        condition=IfCondition(use_control))

    # mode=perception は「カメラブリッジ + 認識だけ」。core の BT も UI も上げない
    # (umiusi_stack.sh --perception と同じ)。既定のまま include すると BT が起動する
    only_full = PythonExpression(["'true' if '", mode, "' == 'full' else 'false'"])
    autonomy = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution(
            [FindPackageShare("umiusi_autonomy"), "launch", "core_autonomy.launch.py"])),
        launch_arguments={"model_path": model_path, "rtsp_url": rtsp_url,
                          "use_camera_bridge": "true",
                          "use_core": only_full,
                          "use_rosbridge": PythonExpression(
                              ["'true' if '", mode, "' == 'full' and '", use_ui, "' == 'true' "
                               "else 'false'"])}.items(),
        condition=_mode_is(mode, *PERCEPTION_MODES))

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
        DeclareLaunchArgument("cameras_param_file", default_value="",
                              description="空なら同梱の cameras_deploy.yaml。実機既定の "
                                          "/dev/video2 は H264 非対応 (known_issues B-1)"),
        DeclareLaunchArgument("use_ui", default_value="true",
                              description="false で rosbridge を上げない (CPU を空ける)。"
                                          "mode=perception では常に上げない"),

        control,
        wait_control,
        RegisterEventHandler(OnProcessExit(
            target_action=wait_control, on_exit=[autonomy, wait_percep, rl_after_control])),
        RegisterEventHandler(OnProcessExit(
            target_action=wait_percep, on_exit=[rl_after_percep])),
    ])
