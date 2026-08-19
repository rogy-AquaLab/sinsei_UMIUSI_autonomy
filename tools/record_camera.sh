#!/usr/bin/env bash
# 実機カメラの映像を RTSP から録画する (デコード・再エンコードなし)。
#
# 実機カメラは gst_camera_node が RTSP に流すだけで ROS トピックを出さないため、
# rosbag だけでは映像が残らない。またカメラデバイス (/dev/videoN) は既に
# gst_camera_node が掴んでいて二重に開けないので、**RTSP 側から録れる**必要がある。
#
#   ./record_camera.sh                       # 既定 (cam1) を録画。Ctrl-C で停止
#   ./record_camera.sh --url rtsp://localhost:8554/cam2 --dir ~/rec
#   ./record_camera.sh --raw                 # 切り捨てに強い生 H264 で保存
#
# 実測: CPU 15.5% / 800x600@15 で約 1.25 MB/s (4.3 GB/時)。
set -o pipefail

URL="${UMIUSI_RTSP_URL:-rtsp://localhost:8554/cam1}"
OUTROOT="${UMIUSI_REC_DIR:-$HOME/recordings}"
SEG_SEC=30
RAW=false

while [ $# -gt 0 ]; do
  case "$1" in
    --url) URL="$2"; shift 2 ;;
    --dir) OUTROOT="$2"; shift 2 ;;
    --seg) SEG_SEC="$2"; shift 2 ;;
    --raw) RAW=true; shift ;;
    *) echo "使い方: $0 [--url URL] [--dir DIR] [--seg 秒] [--raw]"; exit 1 ;;
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
  echo "segment_sec=$SEG_SEC"
  echo "host=$(hostname)"
} > "$OUT/meta.txt"

if [ "$RAW" = true ]; then
  # 生 H264 バイトストリーム。moov atom が無いので **途中で電源が落ちても再生できる**。
  # あとで  ffmpeg -i cam.h264 -c copy cam.mp4  で mp4 化できる。
  # filesink に直接書くときは Annex-B バイトストリームに揃えること。
  # h264parse の既定は AVC (長さ前置) で、生ファイルにすると start code が無く再生できない。
  SINK="video/x-h264,stream-format=byte-stream,alignment=au ! filesink location=$OUT/cam.h264"
  echo "生 H264 で録画 (切り捨てに強い): $OUT/cam.h264"
else
  # mp4 セグメント。**最後のセグメントは finalize が必要**なので、必ず Ctrl-C か
  # SIGINT で止めること (SIGKILL/SIGTERM だと moov atom が書かれず壊れる)。
  SINK="splitmuxsink location=$OUT/cam_%03d.mp4 max-size-time=$((SEG_SEC*1000000000))"
  echo "mp4 セグメントで録画 (${SEG_SEC}秒ごと): $OUT/"
fi

echo "停止は Ctrl-C (きれいに閉じるため kill -9 は使わないこと)"

# -e で SIGINT 時に EOS を流し、mp4 を正しく閉じる
gst-launch-1.0 -e \
  rtspsrc location="$URL" latency=100 protocols=tcp ! \
  rtph264depay ! h264parse config-interval=1 ! \
  $SINK &
GPID=$!

cleanup() {
  echo ""
  echo "EOS を送って finalize しています..."
  kill -INT "$GPID" 2>/dev/null
  for _ in $(seq 1 100); do kill -0 "$GPID" 2>/dev/null || break; sleep 0.1; done
  kill -0 "$GPID" 2>/dev/null && { echo "応答しないので強制終了 (最終セグメントが壊れる可能性)"; kill -9 "$GPID"; }
  echo "保存先: $OUT"
  ls -la "$OUT" 2>/dev/null | tail -5
}
trap cleanup INT TERM
wait "$GPID"
cleanup
