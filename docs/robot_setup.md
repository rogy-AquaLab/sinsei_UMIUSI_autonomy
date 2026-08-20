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

```bash
cd ~/ros2-ws/src
git clone https://github.com/rogy-AquaLab/sinsei_UMIUSI_autonomy.git
```

4 パッケージのモノレポ (`umiusi_autonomy` / `umiusi_autonomy_msgs` / `umiusi_rl_control` /
`umiusi_rl_control_msgs`)。**`src/` に単独の `umiusi_autonomy_msgs` が残っていると
`Duplicate package names not supported` でビルドが止まる**ので、あれば消す。

## 2. セットアップ (1 コマンド)

```bash
cd ~/ros2-ws/src/sinsei_UMIUSI_autonomy
./tools/setup_robot.sh
```

これで依存の解決からビルドまで済む。**システムのファイルは書き換えない** —
Python の依存はすべて `--user` (`~/.local`) に入る。apt が要るもの (ROS のパッケージ等)
だけ `rosdep` が `sudo apt` を使う。

やっていること:

1. `pip` が無ければ `--user` で入れる
2. `rosdep install` で apt / ROS の依存 (`rclpy` / `cv_bridge` / `python3-opencv` /
   `python3-numpy` / `python3-scipy` ...)
3. **torch を CPU 版で** `~/.local` に。PyPI 既定だと **aarch64 でも CUDA 版を引き
   `nvidia-*` で 4.5 GB を無駄にする**ので `--index-url` を明示する。あわせて
   `setuptools` が 80 以上に上がっていたら戻す (colcon が壊れるため)
4. `umiusi_perception` (検出器 + 風船割り FSM)
5. `colcon build` と import 確認

現状の確認だけしたいとき:

```bash
./tools/setup_robot.sh --check
```

### `umiusi_perception` について

`navigator_node` と `auto_target_generator` も FSM のために必要とする
(`perception_node` だけではない)。

**供給元の `Umiusi_sim` が private のあいだは git から取れない。** その場合は
PC から `packages/perception` を持ってきて渡す:

```bash
./tools/setup_robot.sh --perception ~/perception
```

**`Umiusi_sim` を public にすれば、この引数なしで git から入る** (スクリプトが自動で試す)。

### 検出器 (同梱済み)

風船検出器の重みは**リポジトリに同梱してある**ので、転送は要らない。
未指定なら `models/detector/camp_mix.pt` が使われる。

| ファイル | real_val の F1 | 用途 |
|---|---:|---|
| **`camp_real.pt`** | **0.80** | **実際の水中はこちらが強い。競技はこれ** |
| `camp_mix.pt` (既定) | 0.69 | 両対応。実機で通しの動作確認をしたのはこちら |

競技で切り替えるとき:

```bash
ros2 launch umiusi_autonomy core_autonomy.launch.py \
    model_path:=$(ros2 pkg prefix umiusi_autonomy)/share/umiusi_autonomy/models/detector/camp_real.pt
```

詳細は `umiusi_autonomy/models/detector/README.md`。
RL 姿勢制御のポリシーも `umiusi_rl_control/models/cruise_policy/` に同梱済み。

> **`rosdep install` だけで完結させたい場合** — `rosdep/umiusi.yaml` を登録すれば
> torch も rosdep で入るが、**システム側に `/etc/pip.conf` を置く必要がある**
> (rosdep の pip は sudo で system-wide に入れるため PEP 668 に阻まれる。
> CPU インデックスの指定もルールには書けない)。詳細はそのファイルのコメント。

## 3. ビルド

```bash
cd ~/ros2-ws
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
