#!/usr/bin/env bash
# 単体実験モードの実機確認を一気に通す。
#
#   ./experiment_test.sh                  # perception -> 姿勢制御 -> ロギング
#   ./experiment_test.sh --perception     # 認識だけ
#   ./experiment_test.sh --attitude       # 姿勢制御だけ
#   ./experiment_test.sh --logging        # ロギングだけ
#   ./experiment_test.sh --duration 30    # 各フェーズの計測秒数 (既定 20)
#   ./experiment_test.sh --rec-sec 60     # 録画秒数 (既定 45)
#
# 録画は短くできない。record_run.sh は起動に 10 秒以上かかり (ros2 topic list と
# カメラの立ち上げ)、rosbag2 の discovery も数秒かかるため。短すぎると
# bag に /tf しか入らない (実機で確認済み)。
#
# **スラスタは回さない。** RL は publish=false でしか起動しないので、モータには何も出ない。
# 実際に回す確認は docs/experiment_guide.md 1-4 を手順どおりに (e-stop を手元に置いて) 行うこと。
#
# 判定できるのは「起動する / トピックが出る / ログにエラーが無い」まで。水中挙動・色判別・
# 距離精度は判定できない (docs/competition_checklist.md)。
set -o pipefail
# record_run.sh を & で起こして kill -INT するために必須。非対話シェルが & で起こした子は
# SIGINT を SIG_IGN のまま引き継ぐため、ジョブ制御を有効にしないと停止が届かない
# (docs/known_issues.md / record_run.sh の冒頭コメント)。
set -m

HERE="$(cd "$(dirname "$0")" && pwd)"
STACK="$HERE/umiusi_stack.sh"
WS="${UMIUSI_WS:-$HOME/ros2-ws}"
LOGDIR="${UMIUSI_LOGDIR:-/tmp/umiusi_logs}"
RUNROOT="${UMIUSI_RUN_DIR:-$HOME/runs}"
DURATION=20
REC_SEC=45
DO_PERCEPTION=false; DO_ATTITUDE=false; DO_LOGGING=false

PASS=0; FAIL=0; SKIP=0
ok()   { printf "  \033[32m[OK]\033[0m   %s\n" "$*"; PASS=$((PASS+1)); }
ng()   { printf "  \033[31m[NG]\033[0m   %s\n" "$*"; FAIL=$((FAIL+1)); }
skip() { printf "  \033[33m[--]\033[0m   %s\n" "$*"; SKIP=$((SKIP+1)); }
hdr()  { printf "\n\033[1m== %s ==\033[0m\n" "$*"; }

while [ $# -gt 0 ]; do
  case "$1" in
    --perception) DO_PERCEPTION=true; shift ;;
    --attitude)   DO_ATTITUDE=true; shift ;;
    --logging)    DO_LOGGING=true; shift ;;
    --duration)   DURATION="$2"; shift 2 ;;
    --rec-sec)    REC_SEC="$2"; shift 2 ;;
    *) echo "使い方: $0 [--perception] [--attitude] [--logging] [--duration N] [--rec-sec N]"; exit 1 ;;
  esac
done
if [ "$DO_PERCEPTION" = false ] && [ "$DO_ATTITUDE" = false ] && [ "$DO_LOGGING" = false ]; then
  DO_PERCEPTION=true; DO_ATTITUDE=true; DO_LOGGING=true
fi

# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
[ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"
export PATH="$HOME/.local/bin:$PATH"

REC_PID=""
cleanup() {
  # set -m で起こした record_run.sh は別プロセスグループなので Ctrl-C が届かない。
  # 名前で無差別に kill せず、自分が起こした PID だけを止める。
  [ -n "$REC_PID" ] && kill -INT "$REC_PID" 2>/dev/null && sleep 3
  "$STACK" stop > /dev/null 2>&1
}
trap 'echo ""; echo "中断されました。記録とスタックを停止します"; cleanup; exit 130' INT TERM

# 指定トピックの周期を測って期待値と比べる。bench_rates.py は publisher 数も返すので
# 「0 Hz なのは publisher が居ないからか、遅いだけか」を取り違えない。
bench() {
  local json=/tmp/_exp_bench.json
  if ! python3 "$HERE/bench_rates.py" --duration "$DURATION" --json "$@" > "$json" 2>/dev/null; then
    skip "周期計測に失敗 (bench_rates.py)"; return
  fi
  local res; res=$(python3 - "$json" <<'PY'
import json, sys
want = {"/state/imu": 45.0, "/state/thruster_state_all": 45.0,
        "/front_cam/image_raw": 5.0, "/perception_node/detections": 4.0}
d = json.load(open(sys.argv[1]))
for t in d["topics"]:
    name, r, npub = t["topic"], t["rate_hz"], t["publishers"]
    exp = want.get(name)
    if npub == 0:
        print(f"NG\t{name}: publisher が居ない")
    elif exp is None:
        print(f"SKIP\t{name}: {r:.2f} Hz")
    elif r >= exp:
        print(f"OK\t{name}: {r:.2f} Hz (目安 {exp} 以上)")
    else:
        print(f"NG\t{name}: {r:.2f} Hz (目安 {exp} 以上)")
print(f"SKIP\tCPU 使用 {d['cpu_used_pct']}% / 温度 {d['temp_c']}C")
PY
)
  while IFS=$'\t' read -r verdict msg; do
    case "$verdict" in OK) ok "$msg" ;; NG) ng "$msg" ;; *) skip "$msg" ;; esac
  done <<< "$res"
}

# ログに出ていてほしい / 出ていてほしくない行を確認する
log_has()    { grep -q "$2" "$1" 2>/dev/null && ok "$3" || ng "$4"; }
log_hasnt()  { grep -q "$2" "$1" 2>/dev/null && ng "$4" || ok "$3"; }

phase_pre() {
  hdr "0. 事前確認"
  [ -f "$WS/install/setup.bash" ] && ok "ワークスペースはビルド済み" \
    || { ng "$WS がビルドされていない (colcon build --packages-up-to umiusi_autonomy)"; return 1; }

  local share cam
  share="$(ros2 pkg prefix umiusi_autonomy 2>/dev/null)/share/umiusi_autonomy"
  cam="$share/config/cameras_deploy.yaml"
  if [ -f "$cam" ]; then
    ok "同梱の cameras 設定あり: $cam"
    # 設定が指しているデバイスが本当に H264 を出せるか。挿し順で番号が変わるので毎回見る
    local dev
    dev=$(grep -oE "/dev/video[0-9]+" "$cam" | head -1)
    if [ -n "$dev" ] && command -v v4l2-ctl > /dev/null; then
      if [ ! -e "$dev" ]; then
        ng "$dev が存在しない (cameras_deploy.yaml の device を実機に合わせること)"
      elif v4l2-ctl --device="$dev" --list-formats 2>/dev/null | grep -q H264; then
        ok "$dev は H264 を出せる (cameras_deploy.yaml と一致)"
      else
        ng "$dev は H264 非対応。H264 を出せるのは: $(for d in /dev/video*; do v4l2-ctl --device=$d --list-formats 2>/dev/null | grep -q H264 && echo -n "$d "; done)"
      fi
    else
      skip "v4l2-ctl が無いのでデバイス確認を省略"
    fi
  else
    ng "cameras_deploy.yaml が install されていない (colcon build をやり直すこと)"
  fi

  python3 -c "import umiusi_perception" 2>/dev/null \
    && ok "umiusi_perception を import できる" || ng "umiusi_perception が無い"
  ros2 pkg list 2>/dev/null | grep -qx sinsei_umiusi_control \
    && ok "sinsei_umiusi_control あり" || ng "sinsei_umiusi_control が見つからない"
}

phase_perception() {
  hdr "1. perception 単体 (umiusi_stack.sh start --perception)"
  "$STACK" stop > /dev/null 2>&1
  local out; out="$("$STACK" start --perception 2>&1)"
  echo "$out" | sed 's/^/    | /'

  echo "$out" | grep -q "cameras: " \
    && ok "cameras 設定を渡した" || ng "cameras 設定を渡していない (B-1 を踏む)"
  pgrep -f camera_bridge_node > /dev/null && ok "camera_bridge_node が起動" || ng "camera_bridge_node が起動していない"
  pgrep -f perception_node    > /dev/null && ok "perception_node が起動"    || ng "perception_node が起動していない"
  pgrep -f robot_strategy     > /dev/null \
    && ng "robot_strategy が起動している (use_core:=false が効いていない)" \
    || ok "core の BT は起動していない (use_core:=false)"
  pgrep -f rosbridge_websocket > /dev/null \
    && ng "rosbridge が起動している (use_rosbridge:=false が効いていない)" \
    || ok "UI (rosbridge) は起動していない"

  local log="$LOGDIR/core.log"
  log_has   "$log" "接続しました"        "RTSP に接続できた" "RTSP に接続できていない (mediamtx とカメラを確認)"
  log_hasnt "$log" "software に落とします" "HW デコード経路で開けている" "HW デコード経路が開けず software に落ちている (CPU を食う)"
  log_hasnt "$log" "接続できません"        "接続エラーは出ていない"       "接続できませんが出ている (RTSP に映像が来ていない)"

  bench /front_cam/image_raw /perception_node/detections
  "$STACK" stop > /dev/null 2>&1
}

phase_attitude() {
  hdr "2. 姿勢制御 単体 (umiusi_stack.sh start --attitude) — publish はしない"
  "$STACK" stop > /dev/null 2>&1
  local out; out="$("$STACK" start --attitude 2>&1)"
  echo "$out" | sed 's/^/    | /'

  echo "$out" | grep -q "カメラは上げない" && ok "カメラを上げていない" || ng "カメラを上げてしまっている"
  echo "$out" | grep -q "publish=false"    && ok "publish=false で起動" || ng "publish の状態が想定と違う"
  pgrep -f rl_attitude > /dev/null && ok "rl_attitude_node が起動" || ng "rl_attitude_node が起動していない"
  pgrep -f gst_camera_node > /dev/null && ng "カメラノードが起動している" || ok "カメラノードは起動していない"

  log_has "$LOGDIR/rl.log" "policy loaded" "ポリシーを読み込んだ" "ポリシーを読み込めていない (rl.log を確認)"

  # 目標姿勢を投げて、setpoint の購読者が居るか (トピック名が合っているか) を見る
  local sp=/tmp/_exp_setpoint.log
  timeout 20 python3 "$HERE/set_attitude.py" --level > "$sp" 2>&1
  log_hasnt "$sp" "購読者が居ません" "setpoint を受け取る購読者が居る" "setpoint の購読者が居ない (トピック名を確認)"

  bench /state/imu /state/thruster_state_all

  local rej; rej=$(grep -c "IMU サンプルを破棄" "$LOGDIR/rl.log" 2>/dev/null)
  rej=${rej:-0}
  if [ "$rej" -eq 0 ]; then ok "IMU サンプルの棄却なし"
  elif [ "$rej" -lt 5 ]; then skip "IMU サンプルの棄却 $rej 回 (静置なら許容範囲)"
  else ng "IMU サンプルの棄却が $rej 回 — imu_max_gyro / imu_max_step_deg を緩めることを検討"; fi

  "$STACK" stop > /dev/null 2>&1
}

phase_logging() {
  hdr "3. ロギング (${REC_SEC} 秒録って Ctrl-C 相当で閉じる)"
  "$STACK" stop > /dev/null 2>&1
  "$STACK" start --perception > /dev/null 2>&1   # 記録対象のトピックを出すため

  "$HERE/record_run.sh" --name smoke-test > /tmp/_exp_record.log 2>&1 &
  local rp=$!
  REC_PID=$rp
  sleep "$REC_SEC"
  if kill -INT "$rp" 2>/dev/null; then
    ok "記録プロセスに SIGINT を送った"
  else
    ng "記録プロセスが既に居ない (起動に失敗している)"
  fi
  wait "$rp" 2>/dev/null
  REC_PID=""
  sed 's/^/    | /' /tmp/_exp_record.log

  local d; d=$(ls -1dt "$RUNROOT"/*smoke-test 2>/dev/null | head -1)
  if [ -n "$d" ]; then
    ok "記録ディレクトリ: $d"
    if [ -f "$d/bag/metadata.yaml" ]; then
      ok "bag の metadata あり (そのまま開ける)"
    else
      ng "bag の metadata が無い (record_run.sh --fix で復元できるか確認)"
    fi
    # metadata があるだけでは意味がない。中身が入っているかまで見る
    # (実機で「metadata はあるが /tf しか入っていない」を踏んだ)
    local info msgs
    info=$(ros2 bag info "$d/bag" 2>/dev/null)
    msgs=$(echo "$info" | awk '/^Messages:/{print $2}')
    if [ "${msgs:-0}" -gt 0 ]; then
      ok "bag に $msgs 件のメッセージ"
    else
      ng "bag が空 (rosbag2 が購読する前に止めた? --rec-sec を伸ばす)"
    fi
    local t line c
    for t in /state/imu /perception_node/detections; do
      line=$(echo "$info" | grep -F "Topic: $t ")
      c=$(echo "$line" | sed -n 's/.*Count: \([0-9]*\).*/\1/p')
      if [ "${c:-0}" -gt 0 ]; then
        ok "$t: $c 件"
      else
        ng "$t が bag に入っていない (record_run.sh のトピック選定と discovery を確認)"
      fi
    done
    if ls "$d"/video/* > /dev/null 2>&1; then
      ok "映像ファイルあり: $(du -sh "$d"/video 2>/dev/null | awk '{print $1}')"
    else
      ng "映像ファイルが無い ($d/camera.log を確認)"
    fi
  else
    ng "記録ディレクトリが作られていない"
  fi

  sleep 2
  pgrep gst-launch > /dev/null && ng "gst-launch が孤児として残っている" || ok "孤児プロセスは残っていない"
  "$STACK" stop > /dev/null 2>&1
}

echo "実機テストを開始します (スラスタは回しません)"
echo "  計測 ${DURATION} 秒 / 録画 ${REC_SEC} 秒 / ログ: $LOGDIR"
phase_pre || { echo "事前確認で止まりました"; exit 1; }
[ "$DO_PERCEPTION" = true ] && phase_perception
[ "$DO_ATTITUDE"   = true ] && phase_attitude
[ "$DO_LOGGING"    = true ] && phase_logging

hdr "結果"
printf "  OK %s / NG %s / スキップ %s\n" "$PASS" "$FAIL" "$SKIP"
if [ "$FAIL" -eq 0 ]; then
  echo "  自動判定できる範囲では問題なし"
else
  echo "  NG は docs/known_issues.md と突き合わせること。ログ: $LOGDIR/{control,core,rl}.log"
fi
exit $([ "$FAIL" -eq 0 ] && echo 0 || echo 1)
