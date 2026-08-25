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

# 1 本ぶんの録画を**監視つきで**起動し、監視シェルの PID を $PIDS に足す。
# $1=RTSP URL, $2=接頭辞 (cam1/cam2)
#
# gst-launch を裸で起動してはいけない: rtspsrc は RTSP の途切れ (サーバ再起動・TCP 断) で
# EOS/エラーになり **パイプラインごと黙って終了する**。8/25 の水中 run はこれで録画が bag より
# 4.5 分早く終わり、autonomy 区間がほぼ写らなかった。ここでは
#   * 停止要求 (SIGINT/TERM) 以外で gst が死んだら警告を出して 2 s 後に再起動する
#     (寿命が「録画を止めるまで」= bag と一致する)
#   * 再起動のたび出力ファイルに _rN を付ける — filesink/splitmuxsink は同名を**先頭から
#     上書きする**ので、同じ location で再起動するとそれまでの録画が消える
#   * 起動 15 s 後に出力が 0 バイトのままなら警告する (cam2 が 0 B のまま 15 分回った対策。
#     接続はできてもフレームが来ない場合はパイプラインが生きたまま何も書かない)
# 警告は $tag.log と stderr の両方へ出す (record_run.sh 経由なら camera.log に入り、
# 終了時にそちらの cleanup が表面化する)。
# **$(start_one ...) の形で呼ばないこと**: コマンド置換だと別シェルの子になり、
# 親から wait / kill -INT できず finalize に失敗する。
MAX_FAILS="${UMIUSI_REC_MAX_FAILS:-5}"   # 連続で 0 バイトが続いたら再起動を諦める
MAX_BACKOFF=30                           # 再起動間隔の上限 [s]

start_one() {
  local url="$1" tag="$2"
  (
    # **監視シェルの中でジョブ制御を入れ直す。** bash はフォークしたサブシェルで job control を
    # 無効化するので、ここで起こす `gst-launch &` は非対話シェルの非同期子として SIGINT/SIGQUIT を
    # SIG_IGN で継承してしまう (実測: SigIgn 0x6)。いまは GLib の g_unix_signal_add が SIG_IGN を
    # 上書きするので結果的に止まるが、**スクリプトが依拠してよい性質ではない** (gst を ffmpeg や
    # シェルラッパに差し替えた瞬間に finalize が空振りする)。
    set -m
    stop=false gpid="" fails=0 backoff=2
    trap 'stop=true; [ -n "$gpid" ] && kill -INT "$gpid" 2>/dev/null' INT TERM
    n=0
    while [ "$stop" != true ]; do
      suffix=""; [ "$n" -gt 0 ] && suffix="_r$n"
      if [ "$RAW" = true ]; then
        first="$OUT/$tag$suffix.h264"
        sink="video/x-h264,stream-format=byte-stream,alignment=au ! filesink location=$first"
      else
        first="$OUT/${tag}${suffix}_000.mp4"
        sink="splitmuxsink location=$OUT/${tag}${suffix}_%03d.mp4 max-size-time=$((SEG_SEC*1000000000))"
      fi
      # -e で SIGINT 時に EOS を流し、mp4 を正しく閉じる。
      # shellcheck disable=SC2086
      gst-launch-1.0 -e \
        rtspsrc location="$url" latency=100 protocols=tcp ! \
        rtph264depay ! h264parse config-interval=1 ! \
        $sink >> "$OUT/$tag.log" 2>&1 &
      gpid=$!
      # 0 バイト見張り (1 回だけ)。gst と一緒に死ぬよう子として持つ
      ( sleep 15
        [ -s "$first" ] || echo "$(date +%T) ⚠ $tag: 開始 15 s 後も $first が 0 バイト (フレームが来ていない — RTSP 配信元を確認)" \
          | tee -a "$OUT/$tag.log" >&2 ) &
      wpid=$!
      wait "$gpid"; rc=$?          # trap が入ると wait は戻る -> stop を見て抜ける
      kill "$wpid" 2>/dev/null
      [ "$stop" = true ] && break
      # **バックオフと諦め**。RTSP が恒久的に落ちていると gst は 1 秒未満で死ぬので、固定 2 s だと
      # 15 分の run で 300 回以上再起動し、そのたび 0 バイトの $tag_rN が作られる。停止時の
      # 「0 バイト」報告が数百行になって、気付かせるための警告が逆に埋もれる。
      if [ ! -s "$first" ]; then
        fails=$((fails+1))
      else
        fails=0; backoff=2       # 一度でも録れたら仕切り直す
      fi
      if [ "$fails" -ge "$MAX_FAILS" ]; then
        echo "$(date +%T) ⚠ $tag: $MAX_FAILS 回連続で 1 バイトも録れませんでした。再起動を諦めます (RTSP 配信元を確認)" \
          | tee -a "$OUT/$tag.log" >&2
        break
      fi
      echo "$(date +%T) ⚠ $tag: 録画パイプラインが終了 (rc=$rc)。${backoff} s 後に再起動します (-> ${tag}_r$((n+1)))" \
        | tee -a "$OUT/$tag.log" >&2
      n=$((n+1))
      sleep "$backoff"
      [ "$backoff" -lt "$MAX_BACKOFF" ] && backoff=$((backoff*2))
    done
    # 停止要求後、gst の EOS/finalize を待つ。ただし **待ち切ってはいけない**: 親の cleanup は
    # 10 s で kill -9 に移り、-9 されるのはこの監視シェルなので、その時点で gst が生きていると
    # **孤児として録り続ける** (裸で起動していた頃は -9 が gst に直接当たっていた)。
    # 親より先に諦めて確実に落とす。
    for _ in $(seq 1 80); do kill -0 "$gpid" 2>/dev/null || break; sleep 0.1; done
    kill -9 "$gpid" 2>/dev/null
  ) &
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
