# 実験カード — 当日これだけ見る

接続・電源・ネットワークは省略（既知）。**起動と、こちらで作った部分だけ**。
詳細は `scenario_run.md` / `teleop_gamepad.md` / `robot_setup.md`。

---

## 先に読む — 慌てなくていいもの

**以下は既知の未実装によるもので、故障ではない。** この日に直さない。

| 見えるもの | 正体 |
|---|---|
| `Failed to write Can: Not implemented for ...` が 3 秒おきに延々と出る | CAN 書き込みの一部が未実装。**正常** |
| `high_power_circuit_info` の `voltage` / `current` / `temperature` が `0.0` | メイン電源基板の読み出しが未実装。**ESC 個別の電圧が本物** |
| `low_power_circuit_info` の `headlights` と `indicator_led` が `1` (ERROR) | ソフト側の既知の不具合。ライトの故障ではない |
| IMU のログに `previous async trigger is still in progress` が数分に数回 | **正常** |
| サーボが小さく唸る | CAN の指令レートが低いため。**測定には影響しない** |

## 直ちに止めるもの

- `water_leaked` が `true` になった
- ESC の電圧が **20 V を下回った**
- 異音・異臭・発熱
- 1 基だけ極端に挙動が違う
- **判断に迷った**

そのあと運用担当に連絡する。**止め方は各節の `arm` サービス。**

---

## 0. 最初に 1 回だけ

```bash
cd ~/ros2-ws && git -C src/sinsei_UMIUSI_autonomy log --oneline -1   # ← bag と一緒にメモする
python3 -c "from umiusi_perception.classical import ClassicalController; print('OK')"
```

`OK` が出なければ古典制御が動かない。直しかた:

```bash
cd ~/umiusi_sim && git checkout main && git pull
pip install --no-deps --no-index ~/umiusi_sim/packages/perception
```

通らなければ `export PYTHONPATH=~/umiusi_sim/packages/perception/src:$PYTHONPATH`
（**その窓から launch すること**）。詳細は `robot_setup.md`。

**ビルドし直さない。** どうしても必要なら `rm -rf build/umiusi_autonomy install/umiusi_autonomy`
してから `colcon build --packages-select umiusi_autonomy --symlink-install`。

---

## 1. 推力の符号を確定させる ← **最優先。これが通らないと以降は全部無意味**

2026-09-12 のプール実験で、**指令したモーメントと機体の動きが 3 軸とも逆**だった。
arm すると振れが 2〜3 倍に悪化し、yaw は目標から 180° 離れたところで安定していた。
どの基が反転しているかは**あの bag からは分離できない**（4 基が一斉に動いていたため）。
**1 基ずつ測る。4 基すべて。**

### 1-0. まず**全基を規定姿勢に置いて、4 基の相対関係を見る**

1 基ずつ見る前に、**全部同じ指令を入れて揃っているか**を一度に見る。
サーボが 1 基だけ違う向きに寝ている、という類はここで出る。

```bash
python3 tools/thruster_cmd.py pose                  # 全基 servo 0° / duty 0.1 を 20 s 保持
python3 tools/thruster_cmd.py pose --angle 45       # 全基 45° に寝かせて保持
python3 tools/thruster_cmd.py pose --angle -90      # 全基 真下向き
```

**指令を出す前に「何が起きるはずか」を表示する**ので、それと見比べる:

| 指令 | 起きるはず |
|---|---|
| `pose`（servo 0°・全基 +0.1） | **上から見て右回り**（合力ちょうど 0 の純粋な旋回） |
| `pose --angle 45` | 上昇・右回り・機首が上がる |
| `pose --angle -90` | 下降・機首が下がる（旋回なし） |

見るもの: **4 基のサーボが同じ角度に寝ているか** / **噴流の向きが揃っているか** /
**1 基だけ回っていない・逆を向いていないか**。

### 1-a. つぎに**地上で 1 基ずつ**（水に入れない・押さえなくていい）

```bash
./umiusi_stack.sh start --control-only          # 指令を出すノードを一切上げない
python3 tools/thrust_sign_check.py --ground --dry   # 予測だけ見る
python3 tools/thrust_sign_check.py --ground         # 1 基ずつ回して y/n で答える
```

各基について「servo 0°・+duty のとき噴流がどちらへ出るはず」を出すので、
**ティッシュ / 紙片 / 手をノズルの後ろにかざして**見て y/n で答える。
最後に **`is_forward` の値がそのまま出る**ので yaml に書く。

| 基 | 取り付け | 噴流が出るはず |
|---|---|---|
| lf | 前左舷 | **後ろ左舷** |
| lb | 後ろ左舷 | **後ろ右舷** |
| rb | 後ろ右舷 | **前右舷** |
| rf | 前右舷 | **前左舷** |

4 基すべて +duty なら**合力ちょうど 0 の純粋な旋回（上から見て右回り）**になる配置。
噴流はその反対向きに出る。

> **空回しなので長く回さない。** duty 0.12 / 1 回 3 秒に絞ってある。異音・発熱で止める。

### 1-b. 直したら**水で**確かめる

```bash
./record_run.sh --bag-only --name 20260913-sign-check
python3 tools/thrust_sign_check.py --out ~/runs/sign.json    # 水中・IMU で 1 基ずつ
```

**機体は水に浮かべ、回れる程度に緩く係留する。** 固定すると角速度が出ず判定できない。
所要 約 3 分（4 基 × サーボ 2 通り × ±duty）。

**地上で決まるのは符号だけ**で、バンドルの幾何（取り付け角・位置）が合っているかは
決まらない。**幾何が鏡像なら、符号を合わせても水中でまた逆になる。** そこを見るのが 1-b。

### 出力の読みかた

```
  基        サーボ        内積     |Δω|   判定
  lf        0°    +0.134    0.134   正常
  lb        0°    -0.117    0.117   反転
```

- **水平・垂直の両方が反転** → 推力ベクトルごと（ペラの回転方向 / モータ相 / ESC の逆転設定）
- **垂直だけ反転** → サーボの正方向（`servo_sign`）
- **「配線の入れ替わりを疑う」が出た** → CAN の割り当て（`vesc1..4_id = 124/125/126/127` が lf/lb/rb/rf）

> **`--dry` が出す日本語（「上から見て右回り」等）を目で確かめること。**
> 実際の動きと食い違ったら、反転より先にこちらの座標系の取り違えを疑う。**その場合は直さず連絡。**

### モデルに依存しない裏取り

```bash
python3 tools/thruster_cmd.py steady --duty 0.2 --seconds 8    # 前進が前進か
```

**後退したらバンドルの幾何が疑わしい。ハードを直さずに止めること**
（ハードを直すと sim で学習した RL 方策まで巻き添えになる）。

### 直しかた

**唯一の正は control の `params/controllers.yaml` の `is_forward`。** そこを直せば
autonomy は起動時に読みに行く（両方に書かなくてよい）。

```yaml
thruster_controller_lb:
  ros__parameters:
    is_forward: false
```

現場で当たりを探すなら、**走らせたまま**変えられる:

```bash
ros2 param set /classical_attitude thrust_sign '[1.0,-1.0,1.0,-1.0]'   # lf,lb,rb,rf
```

これは一時的な上書き。当たったら yaml に書く。

---

## 2. 姿勢制御だけを見る

```bash
ros2 launch umiusi_autonomy scenario.launch.py use_perception:=false
ros2 service call /classical_attitude/arm std_srvs/srv/SetBool '{data: true}'
```

**起動しただけでは動かない（DISARMED）。** 見るもの: 手で傾けて**戻るか** /
**発振しないか**（9/12 は 0.5 Hz で発振）/ duty が上限に張り付かないか。

止める:

```bash
ros2 service call /classical_attitude/arm std_srvs/srv/SetBool '{data: false}'
```

> **`ros2 topic pub` で `~/estop` に打っても届かない。** 購読側が latch のため
> `transient_local` で待っており、`ros2 topic pub` の既定（volatile）と QoS が非互換で
> **1 通も配送されない**。`arm` サービスか、launch の窓で Ctrl-C。

### 走らせたまま効くつまみ

```bash
ros2 param set /classical_attitude max_duty 0.4     # cap 0.25 は姿勢誤差 10 度で飽和する (実測)
ros2 param set /classical_attitude hold_yaw false   # yaw の保持だけ切る
ros2 param set /classical_attitude k_v_vert 1.2     # 上下の効きを上げる (既定 0 = 前進項のみ)
```

---

## 3. ゲームパッドで操縦する（姿勢制御を効かせたまま）

```bash
ros2 launch umiusi_autonomy scenario.launch.py use_perception:=false \
    cmd_target_topic:=/cmd/target
ros2 param set /classical_attitude cmd_target_yaw_mode rate   # ← 無いと 11 度しか回れない
ros2 service call /classical_attitude/arm std_srvs/srv/SetBool '{data: true}'
```

**パッド背面のスイッチを `X` にする**（`D` だと添字がズレて、エラー無しに操作が狂う）。
切り替えたら挿し直してブラウザも再読み込み。

| 操作 | 効果 |
|---|---|
| 左スティック 上下 | 前後 |
| 左スティック 左右 | 旋回（倒している間ずっと回る） |
| 右スティック | ロール/ピッチ目標（±17°）。**放すと水平に戻る** |
| 十字 左右 | 横移動 |
| L2 / R2 | 上/下 |

**従来の FF 経路と違い、手を放すと水平・現在方位を保ち続ける**（出力 0 になるのではない）。

水から出した状態で `ros2 topic echo /cmd/target` を見て、倒した軸だけが動くか確認してから。

---

## 4. シナリオを回す

```bash
ros2 launch umiusi_autonomy bringup.launch.py mode:=scenario
ros2 service call /classical_attitude/arm std_srvs/srv/SetBool '{data: true}'
```

上がるもの: `perception_node` / `navigator_node`（FSM）/ `classical_attitude`。
**スラスタを叩くのは `classical_attitude` だけ。**

確認:

```bash
ros2 topic info /cmd/direct/thruster_controller/output_lf   # Publisher count: 1 であること
ros2 topic hz /perception_node/detections                   # 検出が流れていること
```

誤検出でぐるぐる回るときは:

```bash
ros2 param set /perception_node min_confidence 0.5     # 誤検出を絞る
ros2 param set /navigator_node  yaw_rate_scale 0.4     # 旋回をゆっくり
```

> **符号が直っていないと、誤検出が無くても「ぐるぐる回る」。** FSM が風船の方位へ向けと
> 指令しても機体は逆に回り、見失って探索に戻り、また逆に回る。**手順 1 が先。**

---

## 5. 録る

**駆動より先に開始する。**

| 目的 | コマンド |
|---|---|
| 姿勢制御・シナリオ | `./record_run.sh --name <名前>` |
| 符号確認（映像不要） | `./record_run.sh --bag-only --name <名前>` |
| **風船の実写を集める** | `./record_run.sh --name <名前> --vision` ＋ **スタック側に `record_vision:=true`**（下記） |
| **フローの素材** | `./record_run.sh --name <名前> --flow` |

> **`--vision` は「余計なものを録らない」ためではなく、`bag` に画像を足すためのもの。**
> 既定では画像は bag に入らない（映像は RTSP から H264 で別録り）。そして
> **スタックを `record_vision:=true` で上げていないと publisher が居ないので黙って空振りする。**
>
> ```bash
> ros2 launch umiusi_autonomy bringup.launch.py mode:=scenario record_vision:=true
> ./record_run.sh --name 20260913-balloon --vision
> ```
>
> 購読レポートに `/front_cam/image_raw/compressed` が出ているかで確認できる。

**開始 20 秒後に「何を購読できたか」が出る。** `/state/imu` と
`/state/thruster_state_all` が無ければ録れていない — **実験を止められるうちに気付くための表示**。

その場で検品:

```bash
python3 tools/bag_check.py ~/runs/latest/bag
python3 tools/flow_check.py --device /dev/video4     # フローが出るか (機体を並進させながら)
```

回収（**手元の PC から**）:

```bash
scp -r pi@umiusi2.local:runs/latest/ ./20260913/
```

**録っただけでは終わっていない。** 8 月に 1 回、機体の再起動で記録を失っている。

---

## 今日ほしいデータ

| | なぜ |
|---|---|
| **符号確認の bag** | 今日の最重要。**ついでに `thrust_curve_exp` も決まる**（スラスタを回した bag がまだ 1 本も無い） |
| 姿勢制御の bag（符号を直した後） | 発振が消えたかの判定。`run_compare.py` にかける |
| **並進しながらの cam2 映像** | 速度推定の素材。sway 問題はこれ待ち |
| **風船の実写** | 広いプールのものが 0 枚。赤黄青を同じ画角に / 距離を変える / **見上げ** / **風船なしも同量** |

---

## 今日やらないこと

- **ゲインの調整。** duty が飽和している間は比例制御になっていない。符号 → cap → その次。
- **cap 0.4 で RL を回す。** 方策は 0.25 で学習しており分布外。
- **解析。** 持ち帰る。現場では `thrust_sign_check.py` と `flow_check.py` の判定だけ見る。

---

## 迷ったら

- 止める → `ros2 service call /<ノード>/arm std_srvs/srv/SetBool '{data: false}'`
- ノード名は **古典 `classical_attitude` / RL だけ `rl_attitude_node`**（`/rl_attitude` は存在しない）
- 録り直しは安い。**判断に迷ったら止めてよい。**
