#!/usr/bin/env bash
# 走行 1 回ぶんの記録をまとめて開始する (映像 + rosbag)。実験の機会は限られるので、
# 「録り忘れ」と「録りすぎ」の両方を避けるための入口をここに一本化する。
#
#   ./record_run.sh                 # 前後カメラ + 状態/指令/検出の bag
#   ./record_run.sh --name pool-01  # 名前を付ける
#   Ctrl-C で両方をきれいに停止する
#   ./record_run.sh --fix           # 記録済み bag の metadata を後から復元する
#
# 映像は H264 のまま (再エンコードなし)、bag は生画像を含めない。実測の根拠は
# docs/logging.md — `ros2 bag record -a` は容量 200 倍・CPU +30pt・認識 -12% になる。
set -o pipefail

# **`set -m` は必須**。非対話シェルが `&` で起こした子は SIGINT/SIGQUIT を SIG_IGN で
# 引き継ぐ (POSIX)。この状態だと
#   * `ros2 bag record` は SIGINT を無視する (CPython は SIG_IGN のとき既定ハンドラを
#     入れないため) -> SIGKILL に落ちて metadata.yaml が書かれない
#   * `record_camera.sh` の `trap ... INT` も効かない (bash は「入口で無視されていた
#     シグナルは trap できない」) -> SIGKILL され、孫の gst-launch が孤児として録り続ける
# ジョブ制御を有効にすると子は独自のプロセスグループになり SIG_IGN を引き継がないので、
# kill -INT が本来どおり届く。
set -m

HERE="$(cd "$(dirname "$0")" && pwd)"
NAME=""
# record_camera.sh の UMIUSI_REC_DIR (既定 ~/recordings) とは別物。取り違えると
# --fix の走査先と実際の保存先がずれるので、専用の変数にしている。
OUTROOT="${UMIUSI_RUN_DIR:-$HOME/runs}"
BAG_ONLY=false
CAM_ONLY=false
FIX=false

while [ $# -gt 0 ]; do
  case "$1" in
    --name) NAME="$2"; shift 2 ;;
    --dir)  OUTROOT="$2"; shift 2 ;;
    --bag-only) BAG_ONLY=true; shift ;;
    --camera-only) CAM_ONLY=true; shift ;;
    --fix) FIX=true; shift ;;
    *) echo "使い方: $0 [--name 名前] [--dir DIR] [--bag-only|--camera-only] [--fix]"; exit 1 ;;
  esac
done

# --fix: 記録済みの bag で metadata.yaml が欠けているものを reindex して読めるようにする。
# rosbag2 は停止のしかたによって metadata を書かないことがあるが、MCAP は自己記述形式なので
# 中身は失われていない。停止処理に頼らず、後からいつでも直せるようにこれを用意している。
if [ "$FIX" = true ]; then
  n=0
  for b in "$OUTROOT"/*/bag; do
    [ -d "$b" ] || continue
    if [ -f "$b/metadata.yaml" ]; then
      echo "  OK   $b"
    else
      printf "  修復 %s ... " "$b"
      if ros2 bag reindex "$b" > /dev/null 2>&1 && [ -f "$b/metadata.yaml" ]; then
        echo "完了"; n=$((n+1))
      else
        echo "失敗"
      fi
    fi
  done
  echo "$n 件を修復しました"
  exit 0
fi

if [ "$BAG_ONLY" = true ] && [ "$CAM_ONLY" = true ]; then
  echo "--bag-only と --camera-only は同時に指定できません"; exit 1
fi

STAMP=$(date +%Y%m%d-%H%M%S)
OUT="$OUTROOT/${STAMP}${NAME:+-$NAME}"
mkdir -p "$OUT"

# 生画像 (/front_cam/image_raw) は**入れない**。映像は H264 のまま別途録る。
TOPICS="
/state/imu
/state/thruster_state_all
/state/high_power_circuit_info
/state/low_power_circuit_info
/state/main_power_enabled
/state/imu_temperature
/perception_node/detections
/cmd/target
/cmd/direct/thruster_controller/output_lf
/cmd/direct/thruster_controller/output_lb
/cmd/direct/thruster_controller/output_rb
/cmd/direct/thruster_controller/output_rf
/joint_states
/tf
/tf_static
"

PIDS=""
CLEANED=false

# **子プロセスを起こす前に** trap を張る。起動直後〜trap 設定前に Ctrl-C が入ると
# 録画と bag が孤児として回り続けてしまうため。
cleanup() {
  [ "$CLEANED" = true ] && return 0   # trap 経由と wait 後の二重呼び出しを防ぐ
  CLEANED=true
  echo ""
  echo "停止しています..."
  for pid in $PIDS; do kill -INT "$pid" 2>/dev/null; done
  for _ in $(seq 1 150); do
    alive=false
    for pid in $PIDS; do kill -0 "$pid" 2>/dev/null && alive=true; done
    [ "$alive" = false ] && break
    sleep 0.1
  done
  for pid in $PIDS; do kill -0 "$pid" 2>/dev/null && kill -9 "$pid" 2>/dev/null; done
  sleep 1
  # rosbag2 は停止時に metadata.yaml を書かないことがある (実機で再現)。
  # そのままだと `ros2 bag info/play` が "Could not find metadata" で開けないが、
  # MCAP は自己記述形式なので reindex すれば完全に復元できる。ここで済ませておく。
  # ただし**この停止処理まで到達しないことがある**ので、最後に --fix を打つ運用も残すこと。
  if [ -d "$OUT/bag" ] && [ ! -f "$OUT/bag/metadata.yaml" ]; then
    echo "  bag の metadata が無いので reindex します..."
    ros2 bag reindex "$OUT/bag" > "$OUT/reindex.log" 2>&1 \
      && echo "  reindex 完了" || echo "  ⚠ reindex に失敗 (bag/reindex.log を確認)"
  fi
  echo "保存しました: $OUT"
  du -sh "$OUT"/* 2>/dev/null | sed 's/^/  /'
}
trap cleanup INT TERM

echo "記録先: $OUT"

if [ "$BAG_ONLY" != true ]; then
  "$HERE/record_camera.sh" --both --raw --dir "$OUT/video" > "$OUT/camera.log" 2>&1 < /dev/null &
  PIDS="$PIDS $!"
  echo "  映像   : 前後カメラ (H264 そのまま) -> $OUT/video/"
fi

if [ "$CAM_ONLY" != true ]; then
  # 存在しないトピックを渡すと record 自体が起動しないので、実在するものだけに絞る
  avail=$(ros2 topic list 2>/dev/null)
  rec=""
  for t in $TOPICS; do echo "$avail" | grep -qx "$t" && rec="$rec $t"; done
  if [ -z "$rec" ]; then
    echo "  ⚠ 記録対象のトピックが 1 つも見つかりません (スタックは起動していますか?)"
  else
    # shellcheck disable=SC2086
    # `< /dev/null` は必須。`set -m` でジョブ制御が有効なので、この子は別プロセスグループに
    # なる。`ros2 bag record` は「Press SPACE で一時停止」のため **stdin を読みにいく**ので、
    # 端末を継いだままだと **SIGTTIN で停止**する。停止すると wait が返って cleanup が走り、
    # SIGINT は停止中のプロセスに届かないので最後は kill -9 になる (対話シェルで実行すると
    # 必ず踏む。ログには最初の warning 1 行しか残らない)。
    ros2 bag record -o "$OUT/bag" $rec > "$OUT/bag.log" 2>&1 < /dev/null &
    PIDS="$PIDS $!"
    echo "  データ : $(echo $rec | wc -w) トピック -> $OUT/bag/"
  fi
fi

[ -z "$PIDS" ] && { echo "何も起動できませんでした"; exit 1; }

{
  echo "start_wall_iso=$(date -Is)"
  echo "start_unix=$(date +%s.%N)"
  echo "host=$(hostname)"
  echo "name=${NAME:-（無し）}"
} > "$OUT/meta.txt"

echo ""
echo "記録中。停止は Ctrl-C (kill -9 は使わないこと — bag と mp4 が閉じられません)"

for pid in $PIDS; do wait "$pid" 2>/dev/null; done
cleanup
