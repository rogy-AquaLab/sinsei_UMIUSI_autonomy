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
ssh pi@<機体名>.local          # mDNS。引けないときは下の「接続のしかた」へ
ip -d link show can0           # state UP / ERROR-ACTIVE / bitrate 500000
systemctl is-active mediamtx   # active (RTSP サーバ)
cam -l                         # カメラが列挙されること
groups                         # video / gpio / i2c / dialout が含まれること
```

### 接続のしかた

**公式手順は「PC からのインターネット共有 + mDNS」**。ただし mDNS が引けないことが多いので、
netplan に**固定 IP を併記**してある (`docs/known_issues.md` B-10)。用途で使い分ける:

| 場面 | やること | 接続先 |
|---|---|---|
| **Pi をネットに出したい**<br>(`git pull` / apt / pip) | PC で**インターネット共有**を有効化<br>Win: Wi-Fi のプロパティ → 共有タブ → 有線を選択<br>Linux: `sudo nmcli con mod "<有線接続名>" ipv4.method shared` | `ssh pi@<機体名>.local`<br>または DHCP で得た IP |
| **ネットが無い / 現場** | **PC 側に固定 IP を手動設定**<br>Win: `ncpa.cpl` → 有線 → IPv4 → `192.168.137.1` / `255.255.255.0`<br>(ゲートウェイと DNS は**空欄**)<br>Linux: `sudo nmcli con mod "<有線接続名>" ipv4.method manual ipv4.addresses 192.168.137.1/24` | **`ssh pi@192.168.137.2`** |
| 従来どおり | 何もしない (両者リンクローカル) | `ssh pi@<機体名>.local` |

**固定 IP は「追加」であって置き換えではない。** `dhcp4` も `link-local` も残っているので、
インターネット共有も mDNS も従来どおり動く。

> **共有をオンオフした直後は Pi が経路変更に適応するまで数分かかる。**
>
> **HUB やケーブルを替えた直後に mDNS が引けなくなったら**、Windows のネットワーク
> プロファイルが「パブリック」に戻っていないか確認する (パブリックだと mDNS がブロックされる)。
> **管理者 PowerShell**で `Set-NetConnectionProfile -InterfaceIndex <n> -NetworkCategory Private`。
> 詳細と探し方は `known_issues.md` B-10。
>
> 繋がらないときは PC 側のファイアウォールも疑うこと (Linux の PC なら
> `sudo ufw allow in on <有線IF> from 10.42.0.0/24`。これが無いと ROS 2 の DDS が通らない)。
> **SSH だけなら PC 側の ufw は関係ない** (PC からの outbound のため)。

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

検出器と風船割り FSM の実体。`navigator_node` と `auto_target_generator` も FSM のために
必要とする (`perception_node` だけではない)。

スクリプトが git から自動で入れる。手元にソースがある場合はそちらからも入れられる:

```bash
./tools/setup_robot.sh --perception ~/perception
```

### 検出器 (同梱済み)

風船検出器の重みはリポジトリに同梱してある。未指定なら `models/detector/camp_real2.pt`。

評価セットは **旧 real_val 25 枚 + 8/25 プール 46 枚**。`camp_real` の F1 が旧 docs の 0.80 から
下がって見えるのはモデルが劣化したのではなく、実プールの画像が評価に入ったため。

| ファイル | val の F1 | 推奨 conf | 用途 |
|---|---:|---:|---|
| **`camp_real2.pt`** (既定) | **0.80** | **0.4** | **競技はこれ**。8/25 プール実写で誤検出を潰した版 |
| `camp_real.pt` | 0.44 | 0.3 | 旧版。A/B 比較用 |
| `camp_mix.pt` | — | 0.3 | sim 寄り。sim_eval の F1 が最良 |

旧版に切り替えるとき (**`conf_thresh` も checkpoint 側で 0.4 → 0.3 に変わる**ので、
A/B ではモデルと閾値の 2 つが同時に動く点に注意):

```bash
ros2 launch umiusi_autonomy core_autonomy.launch.py \
    model_path:=$(ros2 pkg prefix umiusi_autonomy)/share/umiusi_autonomy/models/detector/camp_real.pt
```

詳細は `umiusi_autonomy/models/detector/README.md`。
RL 姿勢制御のポリシーも `umiusi_rl_control/models/` に同梱済み（既定 `av_cal1_best_rep103`）。

## 3. ビルド

```bash
cd ~/ros2-ws
colcon build --packages-up-to umiusi_autonomy --cmake-args -DCMAKE_BUILD_TYPE=Release
```

## 4. ここから先

起動・記録・トラブル対応は **README** を見ること。このファイルは機体を
「ssh で入って ROS が動く」状態にするまでを扱う。

自動判定できる項目は 1 コマンドで確認できる:

```bash
./tools/acceptance_test.sh
```

CAN / VESC 4 台の ping / カメラ / torch / 周期 / IMU 健全性 を見る。
判定できない項目 (水中挙動・色判別・距離精度) は `competition_checklist.md`。
