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

## 4. どのブランチで組むか

**結論: 全部 `main` で動く。**（2026-09-13 に `chore/comment-diet` を sim の main へ
fast-forward マージして push した。それまでは sim だけブランチ指定が必要だった）

| リポジトリ | 必要なブランチ | main との差 | 理由 |
|---|---|---|---|
| `sinsei_UMIUSI_autonomy` | **`main`** | 0（未コミットの変更あり） | 姿勢制御ノード・FSM・ツール類。**コミットすること** |
| `umiusi_sim` | **`main`** | 0（`chore/comment-diet` と同一） | 機体に入れる `umiusi_perception` wheel はここから。`classical.py` もここ |
| `sinsei_UMIUSI_control` | **`main`** | 作業ブランチが 17 先行（不要） | scenario 経路は `/cmd/direct` を使うので control の logic を通らない。autonomy が読むのは `is_forward` だけで、**これは main にもある**（4 基分） |
| `sinsei_umiusi_msgs` | **`main`** | 作業ブランチが 1 先行 | 差分はサーボ角の単位コメント訂正のみ（`rad` → `DEGREES`）。動作に影響しないが、**誤読の元なのでマージしたい** |
| `sinsei_UMIUSI_core` | **`main`** | 0 | UI の中継 (`manual_target_generator`) はここ |
| `sinsei_UMIUSI_ui` | **`main`** | 0 | ゲームパッド |

---

## umiusi_sim から何を入れるか

古典制御器の本体 `packages/perception/src/umiusi_perception/classical.py` は sim にある。
機体に入れるのは **`packages/perception` の wheel だけ**（simulator も学習コードも入らない）。

```bash
cd ~/umiusi_sim && git checkout main && git pull
pip install --no-deps --no-index ~/umiusi_sim/packages/perception
python3 -c "from umiusi_perception.classical import ClassicalController; print('OK')"
```

**`--no-deps` を省かないこと。** 省くと torch や opencv の解決に行って時間を食う。
古典制御だけなら numpy しか使わない。

入らないときの逃げ道（インストールせずに使う）:

```bash
export PYTHONPATH=~/umiusi_sim/packages/perception/src:$PYTHONPATH
```

**この `export` をした窓から launch すること**（環境変数は窓ごと）。


## control を main のままにしてよい理由

scenario / teleop の指令は `/cmd/direct/thruster_controller/output_*` に出る。
`thruster_controller` は **`/cmd/direct` に publisher が 1 つでも居ると自前の logic を
まるごとスキップする**（`thruster_controller.cpp` の `has_no_thruster_publishers`）ので、
control 側の制御ロジックは一切通らない。

autonomy が control から読むのは `thruster_controller_<pos>` の **`is_forward` だけ**で、
これは main にもある。作業ブランチ `feat/rl-attitude-logic` の 17 コミットは control 内蔵の
RL logic 用で、`/cmd/direct` 経路には要らない。

**未 push / 未 PR のブランチが 6 本あるが、当面どれも不要。** 順序は
`fix/torch-link-leaks-into-camera-node` → `fix/hardware-health-flags` →
`fix/can-servo-update-rate` → RL 依存の 3 本。

---

## 機体で組むときの手順

```bash
# 1. autonomy
cd ~/ros2-ws/src/sinsei_UMIUSI_autonomy && git checkout main && git pull

# 2. 古典制御ライブラリ
cd ~/umiusi_sim && git checkout main && git pull
pip install --no-deps --no-index ~/umiusi_sim/packages/perception

# 3. control / msgs / core は main
for r in sinsei_UMIUSI_control sinsei_umiusi_msgs sinsei_UMIUSI_core; do
  git -C ~/ros2-ws/src/$r checkout main && git -C ~/ros2-ws/src/$r pull
done

# 4. ビルド
cd ~/ros2-ws && colcon build --symlink-install
```

### `colcon build` が `File exists` で落ちるとき

`install/` 側に実体ファイルが残ったまま `--symlink-install` に切り替えると失敗する。
**エラー文はファイル名しか出さないので原因が分かりにくい。** 消して建て直す:

```bash
rm -rf build/umiusi_autonomy install/umiusi_autonomy
colcon build --packages-select umiusi_autonomy --symlink-install
```

`launch/` に**存在しないファイルへの symlink が残っている**ときも同じ症状になる
（`stack.launch.py` で踏んだ）。`build/<pkg>/` ごと消すのが確実。

### 入ったか確かめる

```bash
python3 -c "from umiusi_perception.classical import ClassicalController; print('OK')"
ros2 launch umiusi_autonomy scenario.launch.py --show-args | head -3
git -C ~/ros2-ws/src/sinsei_UMIUSI_autonomy log --oneline -1   # bag と一緒にメモする
```

**最後の 1 行は run のたびに記録すること。** 「どのコードで録った bag か」が後から
分からなくなる。

## 5. ここから先

起動・記録・トラブル対応は **README** を見ること。このファイルは機体を
「ssh で入って ROS が動く」状態にするまでを扱う。

自動判定できる項目は 1 コマンドで確認できる:

```bash
./tools/acceptance_test.sh
```

CAN / VESC 4 台の ping / カメラ / torch / 周期 / IMU 健全性 を見る。
判定できない項目 (水中挙動・色判別・距離精度) は `competition_checklist.md`。
