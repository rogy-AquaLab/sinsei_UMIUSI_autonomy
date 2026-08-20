#!/usr/bin/env bash
# 実機カメラの映像を RTSP から録画する (デコード・再エンコードなし)。
#
# 実機カメラは gst_camera_node が RTSP に流すだけで ROS トピックを出さないため、
# rosbag だけでは映像が残らない。またカメラデバイス (/dev/videoN) は既に
# gst_camera_node が掴んでいて二重に開けないので、**RTSP 側から録れる**必要がある。
#
#   ./record_camera.sh                       # 既定 (cam1=前) を録画。Ctrl-C で停止
#   ./record_camera.sh --both                # 前(cam1)と下(cam2)を同時に録画
#   ./record_camera.sh --url rtsp://localhost:8554/cam2 --dir ~/rec
#   ./record_camera.sh --raw                 # 切り捨てに強い生 H264 で保存
#
# 実測: CPU 15.5% / 800x600@15 で約 1.25 MB/s (4.3 GB/時)。
set -o pipefail
# このスクリプト自体が `&` で起動されたときでも、子が SIGINT を SIG_IGN で
# 引き継がないようにジョブ制御を有効にする (record_run.sh 冒頭の注記を参照)。
set -m

URL="${UMIUSI_RTSP_URL:-rtsp://localhost:8554/cam1}"
OUTROOT="${UMIUSI_REC_DIR:-$HOME/recordings}"
SEG_SEC=30
RAW=false
BOTH=false
URL2="${UMIUSI_RTSP_URL2:-rtsp://localhost:8554/cam2}"

while [ $# -gt 0 ]; do
  case "$1" in
    --url) URL="$2"; shift 2 ;;
    --dir) OUTROOT="$2"; shift 2 ;;
    --seg) SEG_SEC="$2"; shift 2 ;;
    --raw) RAW=true; shift ;;
    --both) BOTH=true; shift ;;
    --url2) URL2="$2"; shift 2 ;;
    *) echo "使い方: $0 [--url URL] [--url2 URL] [--both] [--dir DIR] [--seg 秒] [--raw]"; exit 1 ;;
  esac
done

STAMP=$(date +%Y%m%d-%H%M%S)
OUT="$OUTROOT/$STAMP"
mkdir -p "$OUT"

# 後で rosbag と突き合わせられるよう、開始時刻を残す
{
  echo "start_wall_iso=$(date -Is)"
  echo "start_unix=$(date +%s.%N)"
  echo "rtsp_url=$URL"
  [ "$BOTH" = true ] && echo "rtsp_url2=$URL2"
  echo "segment_sec=$SEG_SEC"
  echo "host=$(hostname)"
} > "$OUT/meta.txt"

if [ "$RAW" = true ]; then
  # 生 H264 バイトストリーム。moov atom が無いので **途中で電源が落ちても再生できる**。
  # あとで  ffmpeg -nostdin -r 15 -i cam.h264 -c copy cam.mp4  で mp4 化できる。
  # filesink に書くときは Annex-B に揃えること。h264parse の既定は AVC (長さ前置) で、
  # そのまま生ファイルにすると start code が無く再生できない。
  echo "生 H264 で録画 (切り捨てに強い): $OUT/"
else
  # mp4 セグメント。**最後のセグメントは finalize が必要**なので、必ず Ctrl-C か
  # SIGINT で止めること (SIGKILL/SIGTERM だと moov atom が書かれず壊れる)。
  echo "mp4 セグメントで録画 (${SEG_SEC}秒ごと): $OUT/"
fi

echo "停止は Ctrl-C (きれいに閉じるため kill -9 は使わないこと)"

PIDS=""
CLEANED=false

# **子プロセスを起こす前に** trap を張る。起動直後〜trap 設定前に Ctrl-C が入ると
# gst-launch が孤児として録り続けてしまうため。
cleanup() {
  [ "$CLEANED" = true ] && return 0   # trap 経由と wait 後の二重呼び出しを防ぐ
  CLEANED=true
  echo ""
  echo "EOS を送って finalize しています..."
  for pid in $PIDS; do kill -INT "$pid" 2>/dev/null; done
  for _ in $(seq 1 100); do
    still=false
    for pid in $PIDS; do kill -0 "$pid" 2>/dev/null && still=true; done
    [ "$still" = false ] && break
    sleep 0.1
  done
  for pid in $PIDS; do
    kill -0 "$pid" 2>/dev/null && { echo "応答しないので強制終了 (最終セグメントが壊れる可能性)"; kill -9 "$pid"; }
  done
  echo "保存先: $OUT"
  ls -la "$OUT" 2>/dev/null | tail -6
}
trap cleanup INT TERM

# 1 本ぶんの録画パイプラインを起動し、PID を $PIDS に足す。$1=RTSP URL, $2=接頭辞 (cam1/cam2)
# **$(start_one ...) の形で呼ばないこと**: コマンド置換だと別シェルの子になり、
# 親から wait / kill -INT できず finalize に失敗する。
start_one() {
  local url="$1" tag="$2" sink
  if [ "$RAW" = true ]; then
    sink="video/x-h264,stream-format=byte-stream,alignment=au ! filesink location=$OUT/$tag.h264"
  else
    sink="splitmuxsink location=$OUT/${tag}_%03d.mp4 max-size-time=$((SEG_SEC*1000000000))"
  fi
  # -e で SIGINT 時に EOS を流し、mp4 を正しく閉じる。
  # stdout/stderr は必ずログへ逃がすこと。開いたままだと $(start_one ...) の
  # コマンド置換が gst-launch の終了まで待ってしまい、2 本目が起動しない。
  # shellcheck disable=SC2086
  gst-launch-1.0 -e \
    rtspsrc location="$url" latency=100 protocols=tcp ! \
    rtph264depay ! h264parse config-interval=1 ! \
    $sink > "$OUT/$tag.log" 2>&1 &
  PIDS="$PIDS $!"
}

if [ "$BOTH" = true ]; then
  echo "  前カメラ $URL  -> $OUT/cam1.*"
  start_one "$URL" cam1
  echo "  下カメラ $URL2 -> $OUT/cam2.*"
  start_one "$URL2" cam2
else
  start_one "$URL" cam
fi

for pid in $PIDS; do wait "$pid"; done
cleanup
