#!/usr/bin/env bash
# UMIUSI 実機スタックの起動/停止/状態確認。
#
# ad-hoc に `timeout N ros2 launch ... &` で起動すると、計測の途中で寿命が切れて
# 「入力より認識周期が高い」といった辻褄の合わない結果になる。起動と停止をここに寄せる。
#
#   ./umiusi_stack.sh start              # 実機構成 (control + core + autonomy)
#   ./umiusi_stack.sh start --no-ui      # UI(rosbridge) を止めて CPU を空ける
#   ./umiusi_stack.sh start --with-rl    # RL 姿勢制御も動かす
#   ./umiusi_stack.sh status
#   ./umiusi_stack.sh stop
#
# 単体実験 (docs/experiment_guide.md):
#   ./umiusi_stack.sh start --attitude               # 姿勢制御だけ (カメラは上げない)
#   ./umiusi_stack.sh start --attitude --no-publish  # スラスタへ出さず計算だけ (ドライ試験)
#   ./umiusi_stack.sh start --attitude --attitude-policy  # 姿勢保持だけのポリシーで
#   ./umiusi_stack.sh start --perception   # カメラブリッジ + perception だけ (BT / UI なし)
# ROS の setup.bash は未定義変数を参照するため set -u は使えない
set -o pipefail

WS="${UMIUSI_WS:-$HOME/ros2-ws}"
# 空 = launch の既定 (同梱の models/detector/camp_real2.pt)。以前は $HOME/models を既定に
# していたが、検出器を同梱した今は新しい機体に存在せず model_path が空振りする。
MODEL="${UMIUSI_MODEL:-}"
# umiusi_perception は pip で入れるのが正 (下記は入っていない場合の暫定フォールバック)
PERCEPTION_SRC="${UMIUSI_PERCEPTION_SRC:-}"
# 空なら同梱の cameras_deploy.yaml を使う (下の start() で解決)
CAMERAS_PARAM="${UMIUSI_CAMERAS_PARAM:-}"
RTSP_URL="${UMIUSI_RTSP_URL:-rtsp://localhost:8554/cam1}"
# RL ポリシーバンドルの上書き (空 = ノード既定の av_cal1_best_rep103)
RL_MODEL="${UMIUSI_RL_MODEL:-}"
BRIDGE_RATE="${UMIUSI_BRIDGE_RATE:-10.0}"   # perception が捌ける値に合わせる (供給過多は逆効果)
PIDFILE=/tmp/umiusi_stack.pids
LOGDIR="${UMIUSI_LOGDIR:-/tmp/umiusi_logs}"

NODES="ros2_control_node gst_camera_node camera_bridge_node perception_node
       auto_target_generator robot_strategy manual_target_generator
       low_power_health_check rosbridge_websocket rl_attitude"

# 段の切り替えを sleep ではなくシグナルで待つ。
# 待っているのは依存関係ではなく起動時の CPU 競合 (Pi で torch を 2 回読む間に
# controller_manager と xacro が走る)。sleep は速い機体では無駄に待ち、遅い機体では
# 足りない。UMIUSI_STAGE_WAIT=sleep で従来の固定待ちに戻せる。
wait_topic() {  # <topic> <timeout> [--best-effort]
  local topic=$1 timeout=$2; shift 2
  if [ "${UMIUSI_STAGE_WAIT:-signal}" = "sleep" ]; then sleep "$timeout"; return; fi
  echo "  待機: $topic (最大 ${timeout}s)"
  ros2 run umiusi_autonomy wait_for_topic \
    --topic "$topic" --timeout "$timeout" --allow-timeout "$@" 2>&1 | tail -1
}

# ログに完了行が出るまで待つ。トピックで待てない段だけに使う (rl の「ポリシーを読み終えた」
# は、それを表すトピックが無い — current_setpoint は torch のロード前に出る)。
# 呼ぶ前にそのログを親シェルで空にしておくこと: 背景プロセス側の > は fork 後に効くので、
# 前回の起動の完了行が残っていると最初の grep がそれを拾う (CPU 負荷下で 300 回中 4 回 再現)
wait_log() {   # <logfile> <regex> <timeout>
  local log=$1 pat=$2 timeout=$3 waited=0
  if [ "${UMIUSI_STAGE_WAIT:-signal}" = "sleep" ]; then sleep "$timeout"; return; fi
  echo "  待機: $(basename "$log") に /$pat/ (最大 ${timeout}s)"
  while [ "$waited" -lt "$timeout" ]; do
    grep -qE "$pat" "$log" 2>/dev/null && { echo "  完了 (${waited}s)"; return; }
    sleep 1; waited=$((waited + 1))
  done
  echo "  警告: ${timeout}s 待っても /$pat/ が出ませんでした。続行します"
}

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
  # 前カメラ (CSI, imx708) を動かすには Raspberry Pi 版 libcamera (/usr/local, v0.7.1) の
  # GStreamer プラグインが要る。このディレクトリは gst の既定探索パスに入っていないので
  # 明示する必要がある。apt の gstreamer1.0-libcamera (0.2.0) は Pi 用 IPA を持たず
  # `Failed to load a suitable IPA library` で失敗するので入れてはいけない。
  for d in /usr/local/lib/aarch64-linux-gnu/gstreamer-1.0 /usr/local/lib/gstreamer-1.0; do
    [ -f "$d/libgstlibcamera.so" ] && export GST_PLUGIN_PATH="$d${GST_PLUGIN_PATH:+:$GST_PLUGIN_PATH}"
  done
  mkdir -p "$LOGDIR"
}

start() {
  local ui=true rl=false mode=full publish=true rl_policy=cruise
  for a in "$@"; do
    case "$a" in
      --no-ui)      ui=false ;;
      --with-rl)    rl=true ;;
      --attitude)   mode=attitude; rl=true ;;
      --perception) mode=perception ;;
      --no-publish) publish=false ;;
      --attitude-policy) rl_policy=attitude ;;
      *) echo "不明な引数: $a"; usage; exit 1 ;;
    esac
  done
  setup_env
  : > "$PIDFILE"

  # 実機の既定 params/cameras.yaml は usb_camera が /dev/video2 (unicam = H264 非対応) を
  # 指しており、pipeline が開けず RTSP に映像が来ない (known_issues B-1)。同梱の
  # cameras_deploy.yaml (/dev/video4) を既定で渡す。UMIUSI_CAMERAS_PARAM で上書きできる。
  local share
  share="$(ros2 pkg prefix umiusi_autonomy 2>/dev/null)/share/umiusi_autonomy"
  if [ -z "$CAMERAS_PARAM" ] && [ -f "$share/config/cameras_deploy.yaml" ]; then
    CAMERAS_PARAM="$share/config/cameras_deploy.yaml"
  fi

  # 姿勢制御だけ見るときはカメラを上げない (CPU を空ける)
  local cams=true
  [ "$mode" = attitude ] && cams=false
  local camargs=(enable_cameras:=$cams)
  if [ "$cams" = true ]; then
    if [ -n "$CAMERAS_PARAM" ]; then
      camargs+=("cameras_param_file:=$CAMERAS_PARAM")
      echo "[control] CAN / IMU / カメラ (cameras: $CAMERAS_PARAM)"
    else
      echo "[control] CAN / IMU / カメラ"
      echo "  警告: cameras 設定を渡していません。実機既定の /dev/video2 は H264 非対応で"
      echo "        カメラが開けません (known_issues B-1)。UMIUSI_CAMERAS_PARAM で指定してください"
    fi
  else
    echo "[control] CAN / IMU (カメラは上げない)"
  fi
  : > "$LOGDIR/control.log"      # 起動ごとに空にする
  setsid nohup ros2 launch sinsei_umiusi_control main.yaml "${camargs[@]}" \
    > "$LOGDIR/control.log" 2>&1 < /dev/null & echo $! >> "$PIDFILE"
  # controller_manager の spawner が終わると /state/imu が出はじめる
  wait_topic /state/imu 20 --best-effort

  local modelargs=()
  [ -n "$MODEL" ] && modelargs+=("model_path:=$MODEL")

  case "$mode" in
    attitude)
      echo "[autonomy] 起動しない (--attitude)"
      ;;
    perception)
      echo "[autonomy] カメラブリッジ + perception のみ (BT / UI なし)"
      : > "$LOGDIR/core.log"      # 起動ごとに空にする
      setsid nohup ros2 launch umiusi_autonomy core_autonomy.launch.py \
        "${modelargs[@]}" use_core:=false use_rosbridge:=false \
        use_camera_bridge:=true rtsp_url:="$RTSP_URL" \
        > "$LOGDIR/core.log" 2>&1 < /dev/null & echo $! >> "$PIDFILE"
      wait_topic /perception_node/detections 35
      ;;
    *)
      echo "[autonomy] core + autonomy (BT / perception / カメラブリッジ$([ "$ui" = true ] && echo " / UI"))"
      : > "$LOGDIR/core.log"      # 起動ごとに空にする
      setsid nohup ros2 launch umiusi_autonomy core_autonomy.launch.py \
        "${modelargs[@]}" use_rosbridge:=$ui \
        use_camera_bridge:=true rtsp_url:="$RTSP_URL" \
        > "$LOGDIR/core.log" 2>&1 < /dev/null & echo $! >> "$PIDFILE"
      wait_topic /perception_node/detections 35
      ;;
  esac

  if [ "$rl" = true ]; then
    echo "[rl] RL 姿勢制御 (publish=$publish、**disarmed で起動**)"
    echo "     arm: ros2 service call /rl_attitude_node/arm std_srvs/srv/SetBool \"{data: true}\""
    echo "     前進させるなら ros2 param set /rl_attitude_node vel_cmd 0.4 (既定 0 = 姿勢保持)"
    # 既定は同梱の av_cal1_best_rep103 (本命 17 次元)。--attitude-policy で姿勢保持専用の
    # att_cal1_best_rep103 (14 次元、フォールバック) に差し替える。
    # UMIUSI_RL_MODEL でバンドルディレクトリを直接指定もできる (A/B 試験用: av_sim2real2_rep103)
    local rlmodel=()
    if [ -n "$RL_MODEL" ]; then
      rlmodel=(-p "model_path:=$RL_MODEL")
      echo "     ポリシー: $RL_MODEL (UMIUSI_RL_MODEL)"
    elif [ "$rl_policy" = attitude ]; then
      local rlshare
      rlshare="$(ros2 pkg prefix umiusi_rl_control 2>/dev/null)/share/umiusi_rl_control"
      if [ -d "$rlshare/models/att_cal1_best_rep103/export" ]; then
        rlmodel=(-p "model_path:=$rlshare/models/att_cal1_best_rep103")
        echo "     ポリシー: att_cal1_best_rep103 (姿勢保持のみ)"
      else
        echo "     警告: att_cal1_best_rep103 が見つかりません。既定ポリシーで起動します"
      fi
    fi
    : > "$LOGDIR/rl.log"      # 先に空にする (wait_log の注記)
    setsid nohup ros2 run umiusi_rl_control rl_attitude_node --ros-args \
      -p control_hz:=50.0 -p publish:=$publish "${rlmodel[@]}" \
      > "$LOGDIR/rl.log" 2>&1 < /dev/null & echo $! >> "$PIDFILE"
    wait_log "$LOGDIR/rl.log" "policy loaded from" 10
  else
    echo "[rl] 起動しない (--with-rl / --attitude で有効)"
  fi
  echo "起動完了。ログ: $LOGDIR"
  status
}

# $PIDFILE の各 PID に $1 を送り、$2 デシ秒だけ全員の終了を待つ。全員消えたら 0 を返す。
signal_and_wait() {
  local sig="$1" ticks="$2" pid alive
  while read -r pid; do [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null && kill "-$sig" "$pid" 2>/dev/null; done < "$PIDFILE"
  for _ in $(seq 1 "$ticks"); do
    alive=false
    while read -r pid; do [ -n "$pid" ] && kill -0 "$pid" 2>/dev/null && alive=true; done < "$PIDFILE"
    [ "$alive" = false ] && return 0
    sleep 0.1
  done
  return 1
}

stop() {
  # まず自分が起動した launch を行儀よく終わらせる。
  # `pgrep -f "ros2 launch"` を無差別に kill -9 すると、このスタックと無関係な
  # launch まで巻き添えにするので使わない。
  #
  # SIGINT -> SIGTERM -> SIGKILL と段階的に上げる。SIGINT だけでは足りない:
  # 非対話シェルが `&` で起こした子は SIGINT を SIG_IGN のまま引き継ぎ、CPython は
  # その場合に既定ハンドラを入れないため、**`ros2 launch` は SIGINT を無視する**
  # (実測: 起動直後の disposition が SIG_IGN)。SIGTERM は SIG_IGN を引き継がず、
  # launch 側も明示的にハンドラを入れるので確実に届く。
  if [ -f "$PIDFILE" ]; then
    # ROS 2 の launch ツリーは片付けに 10 秒以上かかることがあるので余裕をもって待つ
    signal_and_wait INT 100 || signal_and_wait TERM 150 || signal_and_wait KILL 20
    rm -f "$PIDFILE"
  fi
  # 取りこぼした個別ノードだけを名指しで止める
  for n in $NODES; do node_pids "$n" | xargs -r kill -9 2>/dev/null; done
  sleep 2
  echo "停止しました"
}

# ノードのプロセスだけを拾う。`pgrep -f <名前>` は名前がコマンド行に出るだけの
# プロセスも拾ってしまい、stop がそれを kill -9 する。実際に踏みうるのは
# 別端末の `ros2 topic echo /perception_node/detections` と、このスクリプトを
# 起動した親シェル自身。
#   * 実行ファイルとして現れる形 (/<名前> の後ろが空白か行末) だけに絞る
#   * 自分と祖先を除く (pgrep はコマンド行に名前が入った自分自身にマッチする)
self_tree() {
  local p=$$
  while [ "${p:-0}" -gt 1 ] 2>/dev/null; do
    printf '%s\n' "$p"
    p=$(ps -o ppid= -p "$p" 2>/dev/null | tr -d ' ')
  done
}

node_pids() {  # <ノード名>
  local skip
  skip=$(self_tree | paste -sd'|' -)
  pgrep -f "/$1( |\$)" 2>/dev/null | grep -Ev "^(${skip:-0})$" || true
}

status() {
  printf "  %-26s %s\n" "ノード" "プロセス数"
  for n in $NODES; do
    # pgrep -c は 0 件でも "0" を出して exit 1 する。|| echo 0 を足すと二重になる
    c=$(node_pids "$n" | grep -c . || true)
    [ "${c:-0}" -gt 0 ] 2>/dev/null && printf "  %-26s %s\n" "$n" "$c"
  done
  echo "  --"
  # 読めない機体では異常値 (-273200 = 絶対零度) を返すことがあるので素通しにしない
  awk '$1 > -50000 {printf "  CPU 温度: %.1f C\n", $1/1000}' \
      /sys/class/thermal/thermal_zone0/temp 2>/dev/null
  command -v vcgencmd >/dev/null && echo "  $(vcgencmd get_throttled)"
}

usage() {
  cat <<'EOS'
使い方: umiusi_stack.sh {start|stop|restart|status} [オプション]

  --no-ui        UI (rosbridge) を起動しない (CPU を空ける)
  --with-rl      RL 姿勢制御も起動する
  --attitude     姿勢制御の単体実験。カメラを上げず、RL だけ起動する
  --perception   認識の単体実験。カメラブリッジ + perception だけ (BT / UI なし)
  --no-publish   RL の指令をスラスタへ出さず計算だけする (ドライ試験)
  --attitude-policy
                 姿勢保持専用ポリシー att_cal1_best_rep103 (14 次元) を使う。
                 既定は av_cal1_best_rep103 (姿勢+速度指令 17 次元、v_cmd 既定 0)

環境変数: UMIUSI_WS / UMIUSI_MODEL / UMIUSI_RL_MODEL / UMIUSI_CAMERAS_PARAM /
          UMIUSI_RTSP_URL / UMIUSI_LOGDIR / UMIUSI_STAGE_WAIT
          UMIUSI_STAGE_WAIT=sleep で段の待ちを従来の固定秒に戻す (既定 signal)
EOS
}

case "${1:-}" in
  start)  shift; start "$@" ;;
  stop)   stop ;;
  status) status ;;
  restart) stop; shift; start "$@" ;;
  *) usage; exit 1 ;;
esac
