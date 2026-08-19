#!/usr/bin/env bash
# UMIUSI 実機スタックの起動/停止/状態確認。
#
# ad-hoc に `timeout N ros2 launch ... &` で起動すると、計測の途中で寿命が切れて
# 「入力より認識周期が高い」といった辻褄の合わない結果になる。起動と停止をここに寄せる。
#
#   ./umiusi_stack.sh start            # 実機構成 (control + core + autonomy)
#   ./umiusi_stack.sh start --no-ui    # UI(rosbridge) を止めて CPU を空ける
#   ./umiusi_stack.sh start --with-rl  # RL 姿勢制御も動かす
#   ./umiusi_stack.sh status
#   ./umiusi_stack.sh stop
# ROS の setup.bash は未定義変数を参照するため set -u は使えない
set -o pipefail

WS="${UMIUSI_WS:-$HOME/ros2-ws}"
MODEL="${UMIUSI_MODEL:-$HOME/models/camp_mix.pt}"
# umiusi_perception は pip で入れるのが正 (下記は入っていない場合の暫定フォールバック)
PERCEPTION_SRC="${UMIUSI_PERCEPTION_SRC:-}"
CAMERAS_PARAM="${UMIUSI_CAMERAS_PARAM:-}"
RTSP_URL="${UMIUSI_RTSP_URL:-rtsp://localhost:8554/cam1}"
BRIDGE_RATE="${UMIUSI_BRIDGE_RATE:-10.0}"   # perception が捌ける値に合わせる (供給過多は逆効果)
PIDFILE=/tmp/umiusi_stack.pids
LOGDIR="${UMIUSI_LOGDIR:-/tmp/umiusi_logs}"

NODES="ros2_control_node gst_camera_node camera_bridge_node perception_node
       auto_target_generator robot_strategy manual_target_generator
       low_power_health_check rosbridge_websocket rl_attitude"

setup_env() {
  # shellcheck disable=SC1091
  source /opt/ros/jazzy/setup.bash
  # shellcheck disable=SC1091
  source "$WS/install/setup.bash"
  if ! python3 -c "import umiusi_perception" 2>/dev/null; then
    [ -n "$PERCEPTION_SRC" ] && export PYTHONPATH="$PERCEPTION_SRC:${PYTHONPATH:-}" \
      || echo "警告: umiusi_perception が見つかりません (pip install --no-deps <perception>)"
  fi
  export PATH="$HOME/.local/bin:$PATH"
  mkdir -p "$LOGDIR"
}

start() {
  local ui=true rl=false
  for a in "$@"; do
    case "$a" in
      --no-ui)   ui=false ;;
      --with-rl) rl=true ;;
    esac
  done
  setup_env
  : > "$PIDFILE"

  local camargs=(enable_cameras:=true)
  [ -n "$CAMERAS_PARAM" ] && camargs+=("cameras_param_file:=$CAMERAS_PARAM")
  echo "[1/3] control (ハードウェア: CAN / IMU / カメラ)"
  setsid nohup ros2 launch sinsei_umiusi_control main.yaml "${camargs[@]}" \
    > "$LOGDIR/control.log" 2>&1 < /dev/null & echo $! >> "$PIDFILE"
  sleep 20

  echo "[2/3] core + autonomy (BT / perception / カメラブリッジ${ui:+ / UI})"
  setsid nohup ros2 launch umiusi_autonomy core_autonomy.launch.py \
    model_path:="$MODEL" use_rosbridge:=$ui \
    use_camera_bridge:=true rtsp_url:="$RTSP_URL" \
    > "$LOGDIR/core.log" 2>&1 < /dev/null & echo $! >> "$PIDFILE"
  sleep 35

  if [ "$rl" = true ]; then
    echo "[3/3] RL 姿勢制御"
    setsid nohup ros2 run umiusi_rl_control rl_attitude_node --ros-args \
      -p control_hz:=50.0 -p publish:=false \
      > "$LOGDIR/rl.log" 2>&1 < /dev/null & echo $! >> "$PIDFILE"
    sleep 10
  else
    echo "[3/3] RL 姿勢制御: 起動しない (--with-rl で有効)"
  fi
  echo "起動完了。ログ: $LOGDIR"
  status
}

stop() {
  for n in $NODES; do pgrep -f "$n" | xargs -r kill -9 2>/dev/null; done
  pgrep -f "ros2 launch" | xargs -r kill -9 2>/dev/null
  [ -f "$PIDFILE" ] && { xargs -r kill -9 < "$PIDFILE" 2>/dev/null; rm -f "$PIDFILE"; }
  sleep 2
  echo "停止しました"
}

status() {
  printf "  %-26s %s\n" "ノード" "プロセス数"
  for n in $NODES; do
    # pgrep -c は 0 件でも "0" を出して exit 1 する。|| echo 0 を足すと二重になる
    c=$(pgrep -c -f "$n" 2>/dev/null); c=$(echo "${c:-0}" | head -1)
    [ "$c" -gt 0 ] 2>/dev/null && printf "  %-26s %s\n" "$n" "$c"
  done
  echo "  --"
  awk '{printf "  CPU 温度: %.1f C\n", $1/1000}' /sys/class/thermal/thermal_zone0/temp 2>/dev/null
  command -v vcgencmd >/dev/null && echo "  $(vcgencmd get_throttled)"
}

case "${1:-}" in
  start)  shift; start "$@" ;;
  stop)   stop ;;
  status) status ;;
  restart) stop; shift; start "$@" ;;
  *) echo "使い方: $0 {start|stop|restart|status} [--no-ui] [--with-rl]"; exit 1 ;;
esac
