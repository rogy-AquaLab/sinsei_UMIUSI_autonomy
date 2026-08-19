# UMIUSI 実機セットアップ〜起動 手順書

新しい機体 (Raspberry Pi 4 / Ubuntu 24.04 / ROS 2 Jazzy) をゼロから立ち上げて autonomy を
動かすまでの手順。**上から順にコピペで実行できる**ように書いてある。

各手順の「なぜ」と実測値の根拠は `docs/bringup_experiment_2026-08-19.md`、
既知の落とし穴は `docs/known_issues.md` を参照。

> 記法: `[PC]` は手元の作業 PC、`[Pi]` は機体側で実行する。

---

## 0. 前提

| 項目 | 想定 |
|---|---|
| 機体 | Raspberry Pi 4 Model B (4 GB 以上)、Ubuntu 24.04 LTS (64bit) |
| ROS 2 | Jazzy (`ros-jazzy-desktop` もしくは `ros-jazzy-ros-base`) |
| 接続 | PC と Pi を **LAN ケーブルで直結**(ルータ経由でも可) |
| ワークスペース | `~/ros2-ws` |

---

## 1. ネットワークを通す

Pi は DHCP クライアントとして起動する。直結の場合、**PC 側が DHCP サーバ兼 NAT になる必要がある**。

### 1-1. [PC] 有線を共有モードにする

```bash
# 有線インタフェース名を確認 (例: eno1)
ip -br link show

sudo nmcli con mod "Wired connection 1" ipv4.method shared
sudo nmcli con up  "Wired connection 1"
```

PC に `10.42.0.1/24` が付き、内蔵 dnsmasq が `10.42.0.10〜254` を配る。

### 1-2. [PC] ファイアウォールを開ける ← 忘れやすい

`ufw` が有効だと **ROS 2 の DDS が全く通らない**(トピックが一切見えない)。

```bash
sudo ufw allow in on eno1 from 10.42.0.0/24
```

有線インタフェース + Pi のサブネット限定なので、Wi-Fi 側やインターネット側は塞がったまま。

### 1-3. [PC] Pi のアドレスを見つける

```bash
ip neigh show dev eno1                    # MAC 2c:cf:67:* が Raspberry Pi
getent hosts <ホスト名>.local              # mDNS でも引ける
journalctl --since "10 min ago" | grep DHCPACK   # ホスト名も分かる
```

### 1-4. [Pi] 疎通確認

```bash
ping -c3 10.42.0.1        # PC へ
ping -c3 8.8.8.8          # NAT 経由でインターネットへ
```

> **アドレスを固定したい場合** — [PC] で
> ```bash
> sudo mkdir -p /etc/NetworkManager/dnsmasq-shared.d
> echo 'dhcp-host=<PiのMAC>,10.42.0.135' | sudo tee /etc/NetworkManager/dnsmasq-shared.d/umiusi.conf
> sudo nmcli con down "Wired connection 1"; sudo nmcli con up "Wired connection 1"
> ```
>
> **作業後に元へ戻す** — `sudo nmcli con mod "Wired connection 1" ipv4.method auto`

---

## 2. [Pi] OS 側の下ごしらえ

### 2-1. CAN (MCP2515) を有効化

`/boot/firmware/config.txt` に以下があること。無ければ追記して再起動。

```
dtparam=spi=on
dtparam=i2c_arm=on
dtoverlay=mcp2515-can0,oscillator=16000000,interrupt=20
```

確認:

```bash
dmesg | grep -i mcp251        # "MCP2515 successfully initialized"
ip -d link show can0          # state UP / ERROR-ACTIVE / bitrate 500000
sudo apt install -y can-utils
candump can0                  # VESC(ATD) のフレームが流れるはず
```

> `ip -d link` が `clock 8000000` と出るのは**正常**。ドライバは発振子の 1/2 を CAN クロックとして
> 報告する (TQ = 2·BRP/Fosc)。`oscillator=16000000` と矛盾しない。

### 2-2. リアルタイム優先度を許可

これが無いと `ros2_control` が
`Could not enable FIFO RT scheduling policy` を出して通常優先度で回る。

```bash
sudo tee -a /etc/security/limits.conf <<'EOF'
pi  -  rtprio  99
pi  -  memlock unlimited
EOF
# 反映には再ログインが必要
```

### 2-3. カメラ関連

```bash
sudo apt install -y gstreamer1.0-libcamera v4l-utils
```

`libcamerasrc` が無いと前方 CSI カメラが FATAL で落ちる。

**USB カメラのデバイス番号を必ず実機で確認すること**:

```bash
for d in /dev/video*; do echo "== $d"; v4l2-ctl --device=$d --list-formats 2>/dev/null | grep "\["; done
```

`v4l2src ! video/x-h264` を使うので、**`H264` を持つノード**を選ぶ
(前回の機体では `/dev/video4`。`video2` は CSI の unicam で H264 非対応)。
選んだ番号を `sinsei_umiusi_control/params/cameras.yaml` の `usb_camera` に設定する。

RTSP の送り先サーバ (`rtsp://localhost:8554/...`) は **別途用意が必要**
(`gst_camera_node` はサーバを起動しない)。

### 2-4. pip を入れる

Ubuntu 24.04 の Pi イメージには `pip` も `ensurepip` も無い。

```bash
sudo apt install -y python3-pip || {
  curl -sSL -o /tmp/get-pip.py https://bootstrap.pypa.io/get-pip.py
  python3 /tmp/get-pip.py --user --break-system-packages
}
echo 'export PATH="$HOME/.local/bin:$PATH"' >> ~/.bashrc
export PATH="$HOME/.local/bin:$PATH"
```

---

## 3. [Pi] ワークスペースを作る

```bash
mkdir -p ~/ros2-ws/src && cd ~/ros2-ws/src
git clone https://github.com/rogy-AquaLab/sinsei_UMIUSI_control.git
git clone https://github.com/rogy-AquaLab/sinsei_UMIUSI_core.git
git clone https://github.com/rogy-AquaLab/sinsei_UMIUSI_msgs.git
git clone https://github.com/rogy-AquaLab/sinsei_UMIUSI_ui.git

# autonomy は **private リポジトリ**。Pi に GitHub の認証が要る
git clone -b feature/rl-control-split \
    git@github.com:rogy-AquaLab/sinsei_UMIUSI_autonomy.git
```

> **`sinsei_UMIUSI_autonomy` は private なので、Pi に認証情報が無いと clone できない**
> (`fatal: could not read Username for 'https://github.com'`)。次のいずれかが必要:
>
> * Pi に deploy key を置いて SSH で clone する (推奨。読み取り専用にできる)
> * Personal Access Token を使う
> * PC で clone して `rsync -az --exclude=.git <repo> pi@<Pi>:~/ros2-ws/src/` で送る
>
> 他の 4 リポジトリは public なので HTTPS で clone できる。

`sinsei_UMIUSI_autonomy` は 4 パッケージのモノレポで、**`umiusi_autonomy_msgs` も含まれる**
(`umiusi_autonomy` / `umiusi_autonomy_msgs` / `umiusi_rl_control` / `umiusi_rl_control_msgs`)。
別途 clone する必要はない。

> `sinsei_UMIUSI_control` は **必ず `main` の最新**を使うこと
> (ESC 推力符号の修正 PR#307 が入っている)。

依存解決とビルド:

```bash
cd ~/ros2-ws
source /opt/ros/jazzy/setup.bash
rosdep install -i --from-paths src -y --rosdistro jazzy
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release
```

> `sinsei_umiusi_core` は `behaviortree_cpp` を要求する。rosdep が入れられない場合は
> `colcon build --packages-up-to umiusi_autonomy` で回避できる。

> **`src/` に `umiusi_autonomy_msgs` が単独で置かれていないか確認すること。**
> 以前は別リポジトリだったが `sinsei_UMIUSI_autonomy` に取り込んだため、古い環境では
> 両方存在して `colcon build: Duplicate package names not supported` で止まる。
> 単独のほうを消せばよい。

### 検出器チェックポイント (git に入っていない)

`camp_mix.pt` などの検出器の重みは **`umiusi_sim` 側で gitignore されており、
どのリポジトリにも入っていない**。PC から手で持ってくる必要がある。

```bash
# [PC] umiusi_sim/models/perception_learned/ から
rsync -az camp_mix.pt camp_real.pt camp_sim.pt pi@<Pi>:~/models/
```

RL 姿勢制御のポリシーは `sinsei_UMIUSI_autonomy` に入っているので、こちらは clone だけでよい
(`umiusi_rl_control/models/cruise_policy/`)。

---

## 4. [Pi] Python 依存

### 4-1. perception (FSM を含む) — **torch 不要**

`navigator_node` と `auto_target_generator` も FSM のために `umiusi_perception` を必要とする。
FSM 連鎖は numpy + scipy だけで動くので、`--no-deps` で軽く入れられる。

```bash
sudo apt install -y python3-scipy        # 既に入っていることが多い
# umiusi_sim/packages/perception を Pi に配置してから:
python3 -m pip install --user --break-system-packages --no-deps ~/perception
python3 -c "from umiusi_perception.autonomy import BalloonBehavior; print('FSM OK')"
```

> 一時的に済ませるなら `export PYTHONPATH=$HOME/perception/src:$PYTHONPATH` でもよい。

### 4-2. 学習済み検出器を使う場合のみ torch

**PyPI 既定の `torch` は aarch64 でも CUDA 版を引き、`nvidia-*` で 4.5 GB 無駄になる。
必ず CPU 版のインデックスを指定すること。**

```bash
python3 -m pip install --user --break-system-packages --no-cache-dir \
    --index-url https://download.pytorch.org/whl/cpu torch
python3 -m pip install --user --break-system-packages "setuptools<80"   # colcon の制約
python3 -c "import torch; print(torch.__version__)"
```

> `setuptools` が 80 以上になると `colcon-core` が壊れる。torch を入れた後は必ず確認する。

### 4-3. RL 姿勢制御を使う場合

**SB3 の policy zip をそのまま実機で読んではいけない。** numpy 2.x で保存されているため
Pi の numpy 1.26 では `ModuleNotFoundError: numpy._core.numeric` で失敗する。

PC で素形式に書き出したものを使う:

```bash
# [PC] umiusi_sim で
./.venv/bin/python export_policy.py     # → models/cruise_policy/export/{weights.pt,obs_norm.npz}
# [PC] Pi へ転送
rsync -az .../cruise_policy/export/ pi@<Pi>:~/models/cruise_policy_export/
```

実機側は **torch だけ**で推論できる (`tools/policy_infer.py`)。SB3・gymnasium は不要。

---

## 5. 起動

### 5-1. 制御スタック (ハードウェア)

```bash
source ~/ros2-ws/install/setup.bash
ros2 launch sinsei_umiusi_control main.yaml enable_cameras:=true
# カメラを切って CAN だけ見たいとき: enable_cameras:=false
```

確認:

```bash
ros2 topic hz /state/imu                 # 50 Hz 出れば IMU OK
ros2 control list_hardware_components    # can / imu / headlights / indicator_led が active
```

### 5-2. autonomy + core (BT / UI)

```bash
ros2 launch umiusi_autonomy core_autonomy.launch.py \
    model_path:=$HOME/models/camp_mix.pt use_rosbridge:=true
```

> `/cmd/target` は **AUTO モードに入るまで出ない**。core の BT が
> `auto_target_generator` を activate する設計のため、起動直後に出ないのは正常。
> 単体で確認したいときは手動で lifecycle を進める:
> ```bash
> ros2 lifecycle set /auto_target_generator configure
> ros2 lifecycle set /auto_target_generator activate
> ```

### 5-3. RL 姿勢制御ループ

```bash
ros2 run umiusi_rl_control rl_attitude_node --ros-args \
    -p control_hz:=50.0 -p publish:=false      # ← まず publish:=false で確認
```

**安全確認が済むまで `publish:=false`** にしておく。武装解除は `~/estop` / `~/arm`。

### 5-4. UI

UI は nginx で静的配信し、`rosbridge_websocket` (上の `use_rosbridge:=true`) 経由で ROS と話す。
ブラウザから `http://<Piのアドレス>/` を開く。

---

## 6. 動作確認チェックリスト

| # | 確認 | 期待値 | コマンド |
|---|---|---|---|
| 1 | CAN リンク | `ERROR-ACTIVE`、bus-errors 0 | `ip -s -d link show can0` |
| 2 | VESC 4台の応答 | 4台とも PONG | `cansend can0 000011XX#00` (XX=7C..7F) |
| 3 | IMU | 50 Hz、静止時ほぼ 0° | `ros2 topic hz /state/imu` |
| 4 | IMU 傾け | 傾けた分だけ動き、水平に戻ると数度以内に復帰 | `tools/imu_monitor.py` |
| 5 | カメラ | パイプラインが `PLAYING`、ノードが落ちない | `ros2 node list \| grep cam` |
| 6 | perception | 検出が出る | `ros2 topic hz /perception_node/detections` |
| 7 | autonomy | activate 後 `/cmd/target` が 50 Hz | `ros2 topic hz /cmd/target` |
| 8 | 姿勢制御 | `/cmd/direct/...` が 50 Hz | `ros2 topic hz /cmd/direct/thruster_controller/output_lf` |

**VESC の ping 応答確認** (最も手早いハード疎通試験):

```bash
timeout 3 candump can0,00001200:1FFFFFFF &
for hex in 7C 7D 7E 7F; do cansend can0 000011${hex}#00; sleep 0.5; done
# PONG のペイロード先頭バイトが応答元の CAN ID
```

---

## 7. 性能の目安 (Raspberry Pi 4 / 4コア 実測)

| 構成 | CPU アイドル | 制御 50 Hz | 姿勢制御 | perception |
|---|---:|---:|---:|---:|
| control のみ (カメラ無し) | 34% | 50.0 Hz | — | 7.9 Hz |
| **全部入り** (カメラ2本 + perception + RL + UI, 22ノード) | **3.6%** | **50.1 Hz** | **33.8 Hz** | **5.2 Hz** |

- **C++ の `ros2_control` は CPU 飽和下でも 50 Hz を維持する**
- **Python 系 (姿勢制御・perception・画像受信) が先に劣化する**
- 検出器単体の上限は **19.8 Hz** (入力を 30 Hz にした場合)
- 温度は 50°C 程度でスロットルは発生しない。**制約は CPU であって熱ではない**

余裕が足りない場合の調整順:

1. カメラの解像度・fps を下げる (USB の H264 パススルーは 12% と安いが、CSI の
   `videoconvert` + encode は 30% 前後かかる)
2. `perception_node` の `max_rate_hz` を下げる (ただし A-2 のエイリアシングに注意)
3. `rl_attitude_node` の `control_hz` を下げる

---

## 8. 画像を PC から流し込む (開発・試験用)

実機カメラは RTSP に流すだけで **ROS トピックに画像を出さない**ため、perception の試験には
PC 側から画像を送るのが手っ取り早い。

```bash
# [PC]
python3 tools/sim_image_pub.py --dir <画像ディレクトリ> --rate 15 --width 320 --height 240
```

- **QoS は RELIABLE にすること**。BEST_EFFORT だと `perception_node` の購読と噛み合わず
  `No messages will be received` になる
- **640×480 は転送が 6.5 Hz で頭打ちになる**。320×240 なら 30 Hz 出る
