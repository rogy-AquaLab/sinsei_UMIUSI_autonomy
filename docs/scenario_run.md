# シナリオを回す — 認識 + FSM を姿勢制御の上に載せる

`bringup.launch.py mode:=scenario` の手順と、**いまどこまで動くか**。

競技シナリオ（探索 → 接近 → 突入）を、姿勢の安定化と 4 基への配分を姿勢制御器に任せて
回す経路。従来の `mode:=navigator` との違いは**そこだけ**で、カメラも検出器も FSM も同じもの。

---

## いまどこまで動くか

**すべて偽機体 (PC 上の剛体シミュレータ) と既知素材での検証。実機では未確認。**

| | 状態 | 根拠 |
|---|---|---|
| `mode:=scenario` の起動とノード構成 | **動く** | `/cmd/direct` の publisher がちょうど 1 つ（姿勢制御器のみ）。navigator は `setpoint` にしか出さない |
| FSM → 姿勢制御器 → スラスタ の閉ループ | **動く** | 風船を +30° 方向に置くと機体が +23.8° まで旋回して追尾 |
| UI ゲームパッドのテレオペが姿勢制御の上に乗る | **動く** | `/cmd/target` に yaw +60° → +60.0° に収束 |
| 反転を `ros2 param set` で直す | **動く** | 反転状態で yaw +90°→+176°（180° の罠を再現）→ `thrust_sign` 設定後は誤差 0°、中央値 0.0° |
| 極性を control の yaml から読む | **動く** | `is_forward` を lb/rf だけ false → `thrust_sign=[1,-1,1,-1]`。control 不在時は自ノードの値へ落ちて警告 |
| **実機の推力の符号** | **未測定・最大の未解決** | 2026-09-12 の bag では反転していた。`tools/thrust_sign_check.py` で 1 基ずつ測るまで確定しない |
| 深度（heave）を追う | **動く（前進項のみ）** | `umiusi_perception.classical` に heave を実装。指令 0・既定ゲインなら従来と完全に同一（浮力トリム −0.1561）。フィードバックは `k_v_vert` 既定 0 で切ってある |
| UI のゲームパッドで旋回し続ける | **動く** | `cmd_target_yaw_mode:=rate` で左スティック −0.2 を 30 秒 → −313°（`absolute` だと −11° で止まる） |
| 広いプールでの認識 | **未確認** | 検出器 `camp_real2.pt` は配備済みだが、広いプールの実写は 0 枚 |
| 突入・破裂の判定（RAM / CONFIRM / POP） | **実機未検証** | sim でしか回っていない |
| RL 姿勢制御との組み合わせ | **未** | scenario は `classical_attitude` 前提。RL は 2026-09-12 に一度も arm されていない |

> **符号が確定するまでシナリオの良し悪しは判定できない。** 反転したままだと、FSM が風船の
> 方位へ向けと指令しても機体は逆に回り、見失って探索に戻り、また逆に回る — **誤検出が
> 無くても「ぐるぐる回る」症状が出る。** 先に `thrust_sign_check.py`。

---

## 手順

### 1. 符号を先に確定させる（**これが通らないと以降は無意味**）

機体を水に浮かべ、**回れる程度に緩く係留**する。固定すると角速度が出ず判定できない。

```bash
./umiusi_stack.sh start --control-only        # 指令を出すノードを一切上げない

# まず地上で (水に入れない・押さえ不要)。噴流の向きを目視して y/n で答える
python3 tools/thrust_sign_check.py --ground --dry
python3 tools/thrust_sign_check.py --ground

# 直したら水で (IMU で 1 基ずつ。幾何が合っているかはここでしか分からない)
./record_run.sh --bag-only --name 20260913-sign-check
python3 tools/thrust_sign_check.py --out ~/runs/sign.json
```

出力の日本語（「上から見て右回り」等）が**実際の動きと合っているか目で確かめること**。
食い違うなら、反転よりも先にこちらの座標系の取り違えを疑う — その場合は直さずに連絡。

モデルに依存しない確認も併せて:

```bash
python3 tools/thruster_cmd.py steady --duty 0.2 --seconds 8   # 前進が前進か
```

**後退したらバンドルの幾何が疑わしい。** ハードを直さずに止めること（ハードを直すと
sim で学習した RL 方策まで巻き添えになる）。

### 2. 反転を直す

**唯一の正は control の `params/controllers.yaml`。** そこの `is_forward` を直すと
autonomy は起動時に読みに行くので、両方に書く必要はない。

```yaml
thruster_controller_lb:
  ros__parameters:
    is_forward: false     # この基は逆
```

現場で試行錯誤するなら、走らせたまま変えられる:

```bash
ros2 param set /classical_attitude thrust_sign '[1.0,-1.0,1.0,-1.0]'
```

`ros2 param set` は**一時的な上書き**で、再起動すると control 側の値に戻る。
当たりが出たら control の yaml に書くこと。

### 3. シナリオを上げる

```bash
ros2 launch umiusi_autonomy bringup.launch.py mode:=scenario
```

立ち上がるもの: `perception_node`（検出）/ `navigator_node`（FSM、`command_mode=setpoint`）/
`classical_attitude`（安定化 + 配分）。**`/cmd/direct` に出すのは姿勢制御器だけ**なので
競合しない。

確認:

```bash
ros2 node list                                        # 上の 3 つが居ること
ros2 topic info /cmd/direct/thruster_controller/output_lf   # Publisher count: 1
ros2 topic info /classical_attitude/setpoint          # Publisher 1 / Subscription 1
ros2 topic hz /perception_node/detections             # 検出が流れていること
```

**起動しただけでは動かない（DISARMED）。** arm はサービスで明示的に:

```bash
ros2 service call /classical_attitude/arm std_srvs/srv/SetBool '{data: true}'
```

### 4. 止める

```bash
ros2 service call /classical_attitude/arm std_srvs/srv/SetBool '{data: false}'
```

**`ros2 topic pub` で `~/estop` に打っても届かない** — 購読側が latch のため
`transient_local` で待っており、`ros2 topic pub` の既定（`volatile`）とは QoS が非互換で
1 通も配送されない。打つなら `--qos-durability transient_local` が要る。
**`arm` サービスを使うのが確実。** launch の窓で Ctrl-C も効く。

> navigator を止めても機体は止まらない。**姿勢制御器が現在方位を保ち続ける。**
> 止めるのは上の `arm` サービス。

---

## 現場で効くパラメータ（すべて走らせたまま変更可）

```bash
ros2 param set /classical_attitude hold_yaw false        # yaw の保持だけ切る
ros2 param set /classical_attitude max_duty 0.4          # 上限を上げる
ros2 param set /perception_node   min_confidence 0.5     # 誤検出を絞る
ros2 param set /navigator_node    yaw_rate_scale 0.4     # 旋回をゆっくりに
```

| パラメータ | 効く場面 |
|---|---|
| `classical_attitude/thrust_sign` | 反転の切り分け。`[1,-1,1,-1]` のように基ごと |
| `classical_attitude/hold_yaw` | 方位を保たせたくないとき。roll/pitch だけ保つ |
| `classical_attitude/max_duty` | **cap 0.25 は姿勢誤差 10° で duty が飽和する**（実測。0.40 なら 25° まで比例制御が保つ）。ただし飽和の原因が cap でないこともあるので、まず符号 |
| `perception_node/min_confidence` | 誤検出。検出器自体の閾値は重みを読んだ時点で固定されるので、これは**後段の足切り**。上げる方向にしか効かない |
| `navigator_node/yaw_rate_scale` | FSM の旋回指令 → 方位変化率 [rad/s]。既定 0.6 |
| `navigator_node/yaw_lead_max` | 目標方位が実測から先行してよい上限 [rad]。既定 1.05 (60°)。**下げると追随は素直になるが旋回が遅くなる** |

---

## 設計 — なぜ setpoint 経路なのか

従来の 2 経路はどちらも**姿勢の安定化を通らない**:

| モード | 指令の行き先 | 姿勢の安定化 | 配分 |
|---|---|---|---|
| `direct`（従来の既定） | navigator が `/cmd/direct` を直接叩く | **無し** | navigator が自前で持つ |
| `target` | `/cmd/target` → control の C++ FF | **無し** | control 側 |
| **`setpoint`（scenario）** | `AttitudeTarget` → 姿勢制御器 | **有り** | 姿勢制御器（1 箇所） |

`direct` は配分を navigator が自前で持つため、実機固有の設定（推力の向き・サーボの向き・
duty 上限）を**姿勢制御器と navigator の両方に配る**必要があった。実際 `thrust_sign` は
3 ノードに足すことになっている。`setpoint` なら実行者が 1 つになる。

### yaw の積分に歯止めが要る理由

FSM は yaw を**変化率**で出すので、姿勢制御器に渡すには方位へ積分する。ここに 2 つの
歯止めを入れてある:

- **初期値を実測方位に合わせる。** 0（= IMU の基準方位）から始めると、現場での機体の
  向き次第でいきなり大きな誤差になる。
- **実測方位から `yaw_lead_max`（既定 60°）以上離さない。** 機体が追随できないまま
  積分し続けると誤差が 180° に達し、**そこが安定平衡になって出られなくなる**。
  2026-09-12 に踏んだのがまさにこの形（符号反転で 180° に捕まり、区間の 64〜95% を
  誤差 170° 超で過ごした）。

### `/cmd/direct` と control の優先関係

`thruster_controller` は **`/cmd/direct` に publisher が 1 つでも居ると自前の logic を
まるごとスキップする**（`thruster_controller.cpp` の `has_no_thruster_publishers`）。
つまり direct が絶対優先で、姿勢制御器が上がっていれば二重駆動にはならない。
`/cmd/direct` は control の `max_duty` もスルーレート制限も素通りする（known_issues B-12）
ので、**歯止めは指令を出す側が持つ**。

---

## 起動を分ける

シナリオの 3 ノードだけを出し入れしたいときは `scenario.launch.py` を直接使う
（control は別に上げておくこと）。`bringup.launch.py mode:=scenario` はこれを
段階起動 + IMU 待ち付きで呼ぶだけ。

```bash
ros2 launch umiusi_autonomy scenario.launch.py                      # 一式
ros2 launch umiusi_autonomy scenario.launch.py use_perception:=false  # 姿勢制御器だけ (teleop 用)
ros2 launch umiusi_autonomy scenario.launch.py use_attitude:=false    # FSM だけ (RL を使うとき)
ros2 launch umiusi_autonomy scenario.launch.py cmd_target_topic:=/cmd/target  # UI テレオペも受ける
```

## UI のテレオペを姿勢制御の上に乗せる

```bash
ros2 launch umiusi_autonomy scenario.launch.py use_perception:=false \
    cmd_target_topic:=/cmd/target
ros2 param set /classical_attitude cmd_target_yaw_mode rate
ros2 service call /classical_attitude/arm std_srvs/srv/SetBool '{data: true}'
```

**UI が出しているのは正規化したスティック値で、物理単位ではない**
(`sinsei_UMIUSI_ui/src/services/gamepadPublisher.ts`):

| フィールド | UI の値 | この制御器が期待するもの | 橋渡し |
|---|---|---|---|
| `velocity.x` | 左スティック上下 [−1, 1] | 速度 [m/s]（到達速度 ≈ 0.21） | `cmd_target_vel_scale`（負 = 到達速度を 1.0 に対応させる） |
| `velocity.y` | 十字キーで ±0.5 | sway [m/s] | 同上 |
| `velocity.z` | L2/R2 で ±0.3 | heave [m/s] | 同上 |
| `orientation.x/y` | 右スティックに ×0.3 | 絶対角 [rad]（±17°） | そのまま。**倒した角度に傾き、放せば水平に戻る** |
| `orientation.z` | 左スティックに ×−0.2 | 絶対角 [rad]（±11°） | **`cmd_target_yaw_mode:=rate` が要る**。absolute だと 11° しか回れない |

## 既知の穴

- **深度は「速度指令」までで、深度そのものは閉じていない。** heave は実装したが、鉛直の
  速度推定は深度センサではなく**指令からの推測**（`VelocityObserver`）なので、
  フィードバックは既定で切ってある（`k_v_vert=0` = 抗力に対する前進項だけ）。
  目標**深度**を保つには水圧センサが要る（`rl_attitude` の `depth_supervisor` の領分）。
  黄色（1.5 m）へ高さを合わせるには、まず実機で heave の効きを測ること。
- **`Target.orientation` の意味が 3 通りに割れている。control の中ですら 2 通り。**
  `logic/attitude/rl.hpp` は REP-103 の回転ベクトル [rad]（絶対量）、
  `logic/attitude/feed_forward.hpp` は 8×6 の配分行列へ直接入れる**無次元のトルク指令**、
  autonomy の navigator `target` モードは正規化した FSM 指令。UI の ±0.2〜0.3 は
  `ff` の解釈（無次元）に合わせて作られている。`classical_attitude` の `cmd_target_topic`
  は **`rl` の規約**で実装してある（3 つのうち唯一の物理量）。揃えるときの変更箇所は
  下の表。
- **ビルドの残骸で `colcon build` が落ちることがある。** `install` 側に実体ファイルが
  残ったまま `--symlink-install` すると `File exists` で失敗する。
  `rm -rf build/umiusi_autonomy install/umiusi_autonomy` してから建て直す。
- **`umiusi_rl_control/test` はディレクトリ指定だと 0 件しか collect されない。**
  ファイルを個別に指定すれば 81 件通る。CI がディレクトリ指定なら素通りしている。

## `Target.orientation` を `rl` の規約に揃えるときの変更箇所

| 場所 | 今 | 要る変更 |
|---|---|---|
| `sinsei_umiusi_msgs/msg/Target.msg` | 規約がどこにも書いていない（**これが根因**） | コメントで「orientation = REP-103 の回転ベクトル [rad]」を明記 |
| control `logic/attitude/feed_forward.hpp` | 無次元のトルク指令として配分行列へ直入れ | **本質的に衝突する。** rad を受けるなら角度 → トルクの段を挟むか、この logic を退役させる |
| control `logic/attitude/rl.hpp` | 既に回転ベクトル [rad] | **変更なし**（これが基準） |
| autonomy `navigator_node._publish_target` | FSM の正規化指令を `orientation.z` に直入れ | rad へ変換する。ただし **`command_mode:=setpoint` を使えばこの経路自体が不要** |
| `sinsei_UMIUSI_ui` `gamepadPublisher.ts` | 正規化したスティック値 | UI 側を rad/m-per-s にするか、**受け側で橋渡しする**（今は後者。上の表） |
| autonomy `classical_attitude` | 既に rl 規約 + UI 用の橋渡し | **変更なし** |

**`ff` をどうするかが実質の判断。** 他は追従するだけ。当面は
`classical_attitude` 側の橋渡しで動くので、急いで揃える必要はない。

## 関連

| ファイル | 中身 |
|---|---|
| `tools/thrust_sign_check.py` | 1 基ずつ符号を判定 + 配線の入れ替え検出 |
| `tools/run_compare.py` | bag を arm 区間で切り分けて姿勢・発振・飽和を出す |
| `tools/flow_check.py` | オプティカルフローの go / no-go |
| `docs/known_issues.md` | A-11 / A-17 / B-12 / B-13 |
| `docs/thrust_calibration.md` | cap と飽和の実測、経路ごとの推力仮定 |
| `docs/experiment_guide.md` | 単体実験の手順 |
| **`docs/field_card.md`** | **当日これだけ見る 1 枚**（起動と我々が作った部分だけ） |
| `docs/robot_setup.md` | セットアップと**どのブランチで組むか** |
| `docs/teleop_gamepad.md` | ゲームパッドの割り当てと X/D スイッチの罠 |
| `docs/logging.md` | 記録スクリプトの使い分け（冒頭に一覧） |
