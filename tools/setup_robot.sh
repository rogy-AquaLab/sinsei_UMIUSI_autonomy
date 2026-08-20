#!/usr/bin/env bash
# 機体を clone 直後の状態から autonomy が動く状態にする。
#
#   ./tools/setup_robot.sh                    # 全部やる
#   ./tools/setup_robot.sh --perception <dir> # umiusi_perception をローカルから入れる
#   ./tools/setup_robot.sh --check            # 何もせず現状だけ確認する
#
# **システムのファイルは書き換えない。** Python の依存は全て `--user` (~/.local) に入れる。
# apt が要るもの (ROS のパッケージ等) だけ rosdep が sudo apt を使う。
#
# なぜ pip 設定ファイル (/etc/pip.conf) を使わないか:
#   rosdep の pip インストーラは sudo で system-wide に入れようとするため、PEP 668 の
#   ブロックを外す設定を **システム側に** 置く必要が出てしまう。ここで先に `--user` で
#   入れておけば rosdep は「充足済み」と見なすので、システムを触らずに済む。
set -o pipefail

WS="${UMIUSI_WS:-$HOME/ros2-ws}"
PERCEPTION_SRC=""
CHECK_ONLY=false
TORCH_INDEX="https://download.pytorch.org/whl/cpu"
PERCEPTION_GIT="umiusi_perception @ git+https://github.com/rogy-AquaLab/Umiusi_sim.git#subdirectory=packages/perception"

while [ $# -gt 0 ]; do
  case "$1" in
    --perception) PERCEPTION_SRC="$2"; shift 2 ;;
    --ws) WS="$2"; shift 2 ;;
    --check) CHECK_ONLY=true; shift ;;
    *) echo "使い方: $0 [--perception <dir>] [--ws <dir>] [--check]"; exit 1 ;;
  esac
done

ok(){ printf "  \033[32m[OK]\033[0m   %s\n" "$*"; }
ng(){ printf "  \033[31m[NG]\033[0m   %s\n" "$*"; }
inf(){ printf "  --     %s\n" "$*"; }
hdr(){ printf "\n\033[1m== %s ==\033[0m\n" "$*"; }

PIP="python3 -m pip"
PIPFLAGS="--user --break-system-packages"   # ~/.local に閉じる。システムは触らない

have_py(){ python3 -c "import $1" 2>/dev/null; }

hdr "0. 前提"
[ -d /opt/ros/jazzy ] && ok "ROS 2 Jazzy" || { ng "/opt/ros/jazzy が無い。公式 Wiki の raspi-setup-2 を先に"; exit 1; }
[ -d "$WS/src" ] && ok "ワークスペース $WS" || { ng "$WS/src が無い"; exit 1; }
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
export PATH="$HOME/.local/bin:$PATH"

if $CHECK_ONLY; then
  hdr "現状の確認のみ (--check)"
  for m in numpy scipy cv2 torch umiusi_perception; do
    have_py "$m" && ok "$m" || ng "$m が入っていない"
  done
  python3 -c "import torch,sys; sys.exit(0 if 'cpu' in torch.__version__ else 1)" 2>/dev/null \
    && ok "torch は CPU 版" || inf "torch が CPU 版か未確認 (未導入 or CUDA 版)"
  exit 0
fi

hdr "1. pip"
if $PIP --version >/dev/null 2>&1; then
  ok "pip あり ($($PIP --version 2>/dev/null | awk '{print $2}'))"
else
  inf "pip が無いので --user で入れる"
  curl -sSL -o /tmp/get-pip.py https://bootstrap.pypa.io/get-pip.py \
    && python3 /tmp/get-pip.py --user --break-system-packages >/dev/null 2>&1 \
    && ok "pip を ~/.local に導入" || { ng "pip の導入に失敗"; exit 1; }
fi

hdr "2. apt / ROS の依存 (rosdep)"
# ここは apt なので sudo が要る。ROS の標準手順どおり。
if rosdep install -i --from-paths "$WS/src" -y --rosdistro jazzy 2>&1 | tail -3; then
  ok "rosdep install 完了"
else
  ng "rosdep install が失敗 (初回なら 'sudo rosdep init && rosdep update' が要るかも)"
fi

hdr "3. torch (CPU 版, ~/.local)"
if have_py torch && python3 -c "import torch,sys; sys.exit(0 if 'cpu' in torch.__version__ else 1)" 2>/dev/null; then
  ok "CPU 版 torch は導入済み ($(python3 -c 'import torch;print(torch.__version__)'))"
else
  inf "CPU 版を ~/.local に入れる (PyPI 既定だと aarch64 でも CUDA 版を引き 4.5GB 無駄になる)"
  # shellcheck disable=SC2086
  $PIP install $PIPFLAGS --no-cache-dir --index-url "$TORCH_INDEX" torch 2>&1 | tail -2
  have_py torch && ok "torch $(python3 -c 'import torch;print(torch.__version__)')" || ng "torch の導入に失敗"
fi
# colcon は setuptools<80 を要求する。torch 導入で上がってしまうことがあるので戻す
if python3 -c "import setuptools,sys; from packaging.version import Version; sys.exit(0 if Version(setuptools.__version__) < Version('80') else 1)" 2>/dev/null; then
  ok "setuptools $(python3 -c 'import setuptools;print(setuptools.__version__)') (colcon 互換)"
else
  inf "setuptools が 80 以上なので colcon 互換の版に下げる"
  # shellcheck disable=SC2086
  $PIP install $PIPFLAGS "setuptools<80" >/dev/null 2>&1
  ok "setuptools $(python3 -c 'import setuptools;print(setuptools.__version__)')"
fi

hdr "4. umiusi_perception (検出器 + 風船割り FSM)"
if have_py umiusi_perception; then
  ok "導入済み"
elif [ -n "$PERCEPTION_SRC" ]; then
  # shellcheck disable=SC2086
  $PIP install $PIPFLAGS --no-deps "$PERCEPTION_SRC" 2>&1 | tail -2
  have_py umiusi_perception && ok "ローカルから導入: $PERCEPTION_SRC" || ng "導入に失敗"
else
  inf "git から取得する (Umiusi_sim は public)"
  # shellcheck disable=SC2086
  if $PIP install $PIPFLAGS --no-deps "$PERCEPTION_GIT" 2>&1 | tail -2 && have_py umiusi_perception; then
    ok "git から導入"
  else
    ng "取得に失敗 (ネットワークは繋がっているか?)"
    inf "手元にソースがあるなら --perception <その場所> で入れられる"
  fi
fi

hdr "5. ビルド"
( cd "$WS" && colcon build --packages-up-to umiusi_autonomy --cmake-args -DCMAKE_BUILD_TYPE=Release 2>&1 | tail -3 )

hdr "6. 確認"
# shellcheck disable=SC1091
source "$WS/install/setup.bash" 2>/dev/null
for m in numpy scipy cv2 torch; do have_py "$m" && ok "$m" || ng "$m"; done
have_py umiusi_perception && ok "umiusi_perception" || ng "umiusi_perception (perception と FSM が動かない)"
python3 -c "import umiusi_autonomy.perception_node, umiusi_rl_control.rl_attitude_node" 2>/dev/null \
  && ok "ノードの import" || ng "ノードの import に失敗"

hdr "次にやること"
echo "  * 検出器は同梱のものが既定で使われる (models/detector/camp_mix.pt)。"
echo "    実際の水中は camp_real.pt のほうが強いので、競技では model_path で切り替える。"
echo "  * 受け入れ試験: $WS/src/sinsei_UMIUSI_autonomy/tools/acceptance_test.sh"
