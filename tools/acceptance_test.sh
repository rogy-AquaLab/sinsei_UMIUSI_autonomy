#!/usr/bin/env bash
# 実機の受け入れ試験。docs/competition_checklist.md のうち自動化できる項目を一気に確認する。
#
#   ./acceptance_test.sh            # スタックは自分で起動しておく
#   ./acceptance_test.sh --start    # スタックの起動から行う
#
# 判定できないもの (水中挙動・色判別・距離精度など) はチェックリスト側を見ること。
set -o pipefail

PASS=0; FAIL=0; SKIP=0
ok()   { printf "  \033[32m[OK]\033[0m   %s\n" "$*"; PASS=$((PASS+1)); }
ng()   { printf "  \033[31m[NG]\033[0m   %s\n" "$*"; FAIL=$((FAIL+1)); }
skip() { printf "  \033[33m[--]\033[0m   %s\n" "$*"; SKIP=$((SKIP+1)); }
hdr()  { printf "\n\033[1m== %s ==\033[0m\n" "$*"; }

WS="${UMIUSI_WS:-$HOME/ros2-ws}"
source /opt/ros/jazzy/setup.bash
[ -f "$WS/install/setup.bash" ] && source "$WS/install/setup.bash"
export PYTHONPATH="${UMIUSI_PERCEPTION_SRC:-$HOME/perception/src}:${PYTHONPATH:-}"

[ "${1:-}" = "--start" ] && { "$(dirname "$0")/umiusi_stack.sh" start; sleep 5; }

hdr "1. ハードウェア"
if ip link show can0 >/dev/null 2>&1; then
  st=$(ip -d link show can0 | grep -oE "state [A-Z-]+" | head -1)
  [ -n "$(ip -d link show can0 | grep ERROR-ACTIVE)" ] && ok "can0 $st (正常)" || ng "can0 $st"
  errs=$(ip -s -d link show can0 | grep -A1 "bus-errors" | tail -1 | awk '{print $2}')
  [ "${errs:-0}" = "0" ] && ok "CAN バスエラー 0" || ng "CAN バスエラー $errs 件"
else
  ng "can0 が存在しない (MCP2515 overlay 未設定?)"
fi

if command -v cansend >/dev/null && ip link show can0 >/dev/null 2>&1; then
  found=0
  for hex in 7C 7D 7E 7F; do
    rm -f /tmp/_pong; timeout 2 candump can0,00001200:1FFFFFFF > /tmp/_pong 2>/dev/null &
    CP=$!; sleep 0.4; cansend can0 000011${hex}#00 2>/dev/null; wait $CP 2>/dev/null
    [ -s /tmp/_pong ] && found=$((found+1))
  done
  [ "$found" -eq 4 ] && ok "VESC(ATD) 4 台すべて応答" || ng "VESC 応答 $found/4 台"
else
  skip "can-utils が無いので VESC ping を省略"
fi

ls /dev/video* >/dev/null 2>&1 && ok "video デバイスあり ($(ls /dev/video* | wc -l) 個)" || ng "video デバイスなし"
if command -v v4l2-ctl >/dev/null; then
  h264=$(for d in /dev/video*; do v4l2-ctl --device=$d --list-formats 2>/dev/null | grep -q H264 && echo $d; done | tr '\n' ' ')
  [ -n "$h264" ] && ok "H264 対応デバイス: $h264" || ng "H264 を出せるデバイスが無い (usb_camera が動かない)"
fi
gst-inspect-1.0 libcamerasrc >/dev/null 2>&1 && ok "libcamerasrc あり (前方 CSI カメラ可)" \
  || ng "libcamerasrc なし -> pi_camera は起動しない (apt install gstreamer1.0-libcamera)"

hdr "2. ソフトウェア環境"
python3 -c "import torch" 2>/dev/null && {
  tv=$(python3 -c "import torch;print(torch.__version__)")
  case "$tv" in *cpu*) ok "torch $tv (CPU 版)";; *) ng "torch $tv — CUDA 版が入っている (無駄に 4.5GB)";; esac
} || skip "torch 未導入 (perception を使わないなら可)"
python3 -c "from umiusi_perception.autonomy import BalloonBehavior" 2>/dev/null \
  && ok "umiusi_perception (FSM) を import できる" || ng "umiusi_perception が無い -> FSM が動かない"
[ "${OMP_NUM_THREADS:-}" = "1" ] && ok "OMP_NUM_THREADS=1 (実機で最速)" \
  || skip "OMP_NUM_THREADS 未設定 (launch 側で設定済みなら可)"

hdr "3. 周期 (20 秒計測)"
BENCH="$(dirname "$0")/bench_rates.py"
if [ -f "$BENCH" ]; then
  python3 "$BENCH" --duration 20 --json \
    /state/imu /state/thruster_state_all /front_cam/image_raw \
    /perception_node/detections /cmd/target > /tmp/_bench.json 2>/dev/null
  python3 - <<'PY'
import json
d = json.load(open("/tmp/_bench.json"))
want = {"/state/imu": 45.0, "/state/thruster_state_all": 45.0,
        "/front_cam/image_raw": 5.0, "/perception_node/detections": 4.0}
for t in d["topics"]:
    name, r, npub = t["topic"], t["rate_hz"], t["publishers"]
    if npub == 0:
        print(f"  \033[33m[--]\033[0m   {name}: publisher なし")
    elif name in want and r >= want[name]:
        print(f"  \033[32m[OK]\033[0m   {name}: {r:.2f} Hz (目安 {want[name]} 以上)")
    elif name in want:
        print(f"  \033[31m[NG]\033[0m   {name}: {r:.2f} Hz (目安 {want[name]} 以上)")
    else:
        print(f"  \033[33m[--]\033[0m   {name}: {r:.2f} Hz")
print(f"  CPU 使用 {d['cpu_used_pct']}% / 温度 {d['temp_c']}C")
PY
else
  skip "bench_rates.py が見つからない"
fi

hdr "4. IMU の健全性 (静置して 30 秒)"
G="$(dirname "$0")/imu_glitch.py"
[ -f "$G" ] && python3 "$G" 30 2>/dev/null | tail -6 | sed 's/^/  /' || skip "imu_glitch.py なし"

hdr "結果"
printf "  OK %s / NG %s / スキップ %s\n" "$PASS" "$FAIL" "$SKIP"
[ "$FAIL" -eq 0 ] && echo "  自動判定できる範囲では問題なし" || echo "  NG を docs/known_issues.md と突き合わせること"
exit $([ "$FAIL" -eq 0 ] && echo 0 || echo 1)
