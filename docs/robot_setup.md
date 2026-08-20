# autonomy を実機に載せる手順

**土台となる Pi のセットアップ (OS / ROS 2 / ネットワーク / CAN / GPIO / カメラ / UI) は
RogikenWiki の公式手順が正。** この文書はそこに載っていない **autonomy 固有の差分だけ**を扱う。

- `/laboratory/Aqua/ROV CORE UNIT/program/raspi-setup-1..3` — Pi の環境構築
- `/laboratory/Aqua/ROV CORE UNIT/制御基板` — 基板・CAN・IMU・電源

公式手順を終えた状態 (ROS 2 Jazzy が入り、`can0` が上がり、MediaMTX と nginx が動き、
`ssh pi@<機体名>.local` で入れる) を前提にする。

---

## 0. 前提の確認

```bash
ssh pi@<機体名>.local          # mDNS で入れること (IP は使わない)
ip -d link show can0           # state UP / ERROR-ACTIVE / bitrate 500000
systemctl is-active mediamtx   # active (RTSP サーバ)
cam -l                         # カメラが列挙されること
groups                         # video / gpio / i2c / dialout が含まれること
```

> **ネットワークは「PC からのインターネット共有」が正規手順**。Pi 側の
> `/etc/netplan/99-*.yaml` に `link-local: [ipv4]` があるので、共有の有無に関わらず
> mDNS で引ける。**共有をオンオフした直後は Pi が経路変更に適応するまで数分かかる**。
> 繋がらないときは PC 側のファイアウォールを疑うこと (Linux の PC なら
> `sudo ufw allow in on <有線IF> from 10.42.0.0/24`。これが無いと ROS 2 の DDS が通らない)。

---

## 1. autonomy リポジトリを置く

**`sinsei_UMIUSI_autonomy` は private** なので、Pi に GitHub の認証が無いと clone できない
(`fatal: could not read Username`)。他の 4 リポジトリは public。

```bash
cd ~/ros2-ws/src
git clone git@github.com:rogy-AquaLab/sinsei_UMIUSI_autonomy.git   # deploy key が要る
# 認証を置きたくなければ PC から:
#   rsync -az --exclude=.git <PC の repo> pi@<機体名>.local:~/ros2-ws/src/
```

4 パッケージのモノレポ (`umiusi_autonomy` / `umiusi_autonomy_msgs` / `umiusi_rl_control` /
`umiusi_rl_control_msgs`)。**`src/` に単独の `umiusi_autonomy_msgs` が残っていると
`Duplicate package names not supported` でビルドが止まる**ので、あれば消す。

## 2. Python 依存

公式手順に含まれないもの。**`pip` が無ければ先に入れる** (Ubuntu の Pi イメージには
`python3-pip` も `ensurepip` も無い):

```bash
sudo apt install -y python3-pip || {
  curl -sSL -o /tmp/get-pip.py https://bootstrap.pypa.io/get-pip.py
  python3 /tmp/get-pip.py --user --break-system-packages
}
export PATH="$HOME/.local/bin:$PATH"
```

### 2-1. `umiusi_perception` (FSM を含む。torch 不要)

`navigator_node` と `auto_target_generator` も FSM のためにこれを必要とする。
FSM 連鎖は numpy + scipy だけで動く。

```bash
# umiusi_sim/packages/perception を Pi に置いてから
python3 -m pip install --user --break-system-packages --no-deps ~/perception
python3 -c "from umiusi_perception.autonomy import BalloonBehavior; print('OK')"
```

### 2-2. torch (学習済み検出器を使う場合のみ)

**PyPI 既定の `torch` は aarch64 でも CUDA 版を引き、`nvidia-*` で 4.5 GB 無駄になる。
必ず CPU 版のインデックスを指定すること。**

```bash
python3 -m pip install --user --break-system-packages --no-cache-dir \
    --index-url https://download.pytorch.org/whl/cpu torch
python3 -m pip install --user --break-system-packages "setuptools<80"   # colcon の制約
```

> `setuptools` が 80 以上になると `colcon-core` が壊れる。torch を入れた後は必ず確認する。

### 2-3. 検出器チェックポイント (git に入っていない)

`camp_mix.pt` などは `umiusi_sim` 側で gitignore されており**どのリポジトリにも無い**。
PC から手で持ってくる。

```bash
rsync -az camp_mix.pt pi@<機体名>.local:~/models/
```

RL 姿勢制御のポリシーは autonomy リポジトリに入っているので clone だけでよい。

## 3. ビルド

```bash
cd ~/ros2-ws
rosdep install -i --from-paths src -y --rosdistro jazzy
colcon build --packages-up-to umiusi_autonomy --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## 4. 起動

```bash
~/ros2-ws/src/sinsei_UMIUSI_autonomy/tools/umiusi_stack.sh start          # UI あり
~/ros2-ws/src/sinsei_UMIUSI_autonomy/tools/umiusi_stack.sh start --no-ui  # CPU を空ける
```

`ln -sfn ~/ros2-ws/src/sinsei_UMIUSI_autonomy/tools ~/umiusi-tools` としておくと楽。

> **前カメラ (CSI) を使うには公式手順の環境変数が要る** (`LIBCAMERA_IPA_*` /
> `LD_LIBRARY_PATH` / `GST_PLUGIN_PATH`)。`~/.bashrc` に入っているが、**非対話シェルでは
> `.bashrc` が即 return するため効かない**。systemd やスクリプトから起動する場合は明示すること
> (`umiusi_stack.sh` は `GST_PLUGIN_PATH` を自前で設定している)。
> **apt の `gstreamer1.0-libcamera` は絶対に入れないこと** — Camera Module V3 非対応で、
> `/usr/local` のソースビルド版を隠して `Failed to load a suitable IPA library` になる。

## 5. 確認

```bash
~/umiusi-tools/acceptance_test.sh
```

CAN / VESC 4 台の ping / カメラ / torch / 周期 / IMU 健全性 を自動判定する。
判定できない項目 (水中挙動・色判別・距離精度など) は `docs/competition_checklist.md`。

## 6. 記録

```bash
~/umiusi-tools/record_run.sh --name <走行名>   # 映像 + rosbag。Ctrl-C で停止
~/umiusi-tools/record_run.sh --fix             # 走行後に bag の metadata を復元
```

詳細と実測値は `docs/logging.md`。**`ros2 bag record -a` は使わない** (容量 200 倍)。

## 7. 性能の目安

`docs/performance_tuning.md` に実測値。要点だけ:

- **C++ の `ros2_control` は CPU 飽和下でも 50 Hz を維持**。Python 系が先に劣化する
- **perception は前カメラ (CSI) を使う** — 下カメラ (USB) より速い (7.70 vs 4.73 Hz)
- **torch のスレッドは 1 に固定** (実機では多いほど遅い。launch に設定済み)
- 温度は 50°C 程度でスロットルは出ない。**制約は CPU であって熱ではない**
