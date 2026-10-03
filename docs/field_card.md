# 実験カード — 当日これだけ見る

接続・電源・ネットワークは省略（既知）。**起動と、こちらで作った部分だけ**。
詳細は `scenario_run.md` / `teleop_gamepad.md` / `robot_setup.md`。

---

## 2026-10-03 最後の実験 — 競技の自律を通す

**ブランチ**: 各 repo `develop`（PR マージ後）。exp は使わない。control の yaml は **`control_mode: fb`**（ff だと方位保持が効かずヨーも 0.18 しか出ない）。
lf は 10/01 の 19:47 で回っていた → 地上で確かめて問題なければ 4 基（3 基だと前進が旋回になる）。**1 基止めるなら `esc_disabled` だけ**（`servo_disabled` は全基 0 になる）。

上から順。

1. **起動**: `ros2 launch umiusi_autonomy core_autonomy.launch.py`（core の main.yaml と同時に上げない）→ `preflight.py`。UI で cam1 が映っているか
2. **カメラの向き**（B-24、未確定）: 風船を機体の**右**に置き、`/front_cam/image_raw` でも**右**に映るか / **上**に置いて上に映るか。
   10/01 の映像は横倒し・上下逆に見える。違ったら FSM の左右・上下が入れ替わるので AUTO の前に報告に戻る
3. **カメラが落ちても戻るか**（control #336）: `pkill -f "__node:=pi_camera"` → 2 s 後に UI で cam1 が戻る / bridge の「フレーム中継」ログが再開
4. **認識**: 既定の検出器は **`balloon_F320_20261003.pt` + `min_confidence` 0.40**（umiusi_sim、JAMSTEC 映像込みで学習。
   `models/detector/README.md`）。風船を置いて `view_detections.py` で色が合っているか。誤検出が多ければ
   `ros2 param set /perception_node min_confidence 0.45`（走らせたまま効く。0.5 以上は赤の検出が半分に落ちる）。
   **周期**: `ros2 topic hz /perception_node/detections` が 4 Hz を大きく下回るなら予備の F256 で起動し直す
   （`model_path:=$(ros2 pkg prefix umiusi_autonomy)/share/umiusi_autonomy/models/detector/balloon_F256_20261003.pt`）
5. **MANUAL**: スティック前 → 前進か（逆なら AUTO の `surge_sign` を -1.0 に、6. 参照）/ R1 で方位保持 → ナビバーのコンパスが**緑**（黄なら ff）
6. **AUTO**: 風船なしで SEARCH 旋回 → 風船ありで寄る・突く → **STANDBY で止まる**。
   **風船を機体の右前に置いたら右に回るか**（FSM の yaw → `yaw_rate` の符号は control 経路で実機未確認。
   逆に回って離れていくなら `ros2 param set /auto_target_generator yaw_rate_scale -1.0`、前後が逆なら
   `ros2 param set /auto_target_generator surge_sign -1.0`。毎周期読み直すので走らせたまま効く）。
   **突進を 1 回見る**: FSM は sim の推奨（`ram_surge` 0.6 / `ram_max_steps` 200 / `ki_heave` 0.3）。`ram_surge` は
   **推力の割合**で sim の m/s とは意味が違う。速すぎ・遅すぎなら `ros2 param set /auto_target_generator ram_surge 0.4` など。
   元の値は 0.26 / 85 / 0.0。起動ログに「ki_heave が無い」と出たら Pi の umiusi_perception が古い（wheel を更新）
   AUTO 中にカメラを落とす（3. と同じ）→ 0.5 s で「検出が途切れた」警告が出て SEARCH に戻る（autonomy #36）
7. **記録**: `record_run.sh --vision --name <名前>`。AUTO の run は全部録る

**競技時間で止まる仕組みは無い**。止めるのは UI の STANDBY（と UI 切断時の電源 OFF）だけ。

---

## 2026-10-01 今日やること（本番前の実験はあと 1 回。次回は競技の自律まで回す）

上から順。**1〜3 は次回の自律の前提**、4〜6 は数値とデータ。

0. **サーボの極性**（B-23、地上・STANDBY）: サーボ 0 で噴流の向き / サーボ +90°（`pose --angle 1.571`）で
   **全基とも噴流が下**。**lf は実機で一度も確かめていない**
1. **起動と検品**: 下の組み合わせで起動 → `preflight.py`。MANUAL で
   **スティック前 → 前進**（= AUTO の `surge_sign` の確認を兼ねる）/ 左右ヨーの向き / roll・pitch
2. **方位保持 (R1)**: 保持中に手で押して戻るか / 保持中にスティックで向きを変えられるか / OFF で戻るか
3. **AUTO（競技の自律）**: 風船なしで SEARCH の旋回が出る → 風船を置いて寄る・突く。**STANDBY で止まる**こと
4. **FSM の換算用**（映像 + bag）: ヨーレート（スティック全倒し・半分）/ 前進（0.3・0.6・1.0）の実速度 /
   上下 ±0.3 の開ループ速度（深度センサが無いので、何秒でどれだけ動くかが要る）
5. **機体の数値**（STANDBY で `thruster_cmd.py`）: 定常 duty の階段（`thrust_curve_exp`）/ サーボ 90° ステップ
   （`servo_max_angular_velocity` 4.0 rad/s の裏取り）/ 自由減衰 / バラスト前後の「指令 0」30 秒 / 推力ありの長い IMU
6. **映像**: 風船（赤黄青を同じ画角 / 距離を変える / 見上げ / 風船なしも同量）— mix と real_only のモデル比較にも使う。
   並進しながらの cam2

**やらないこと**: ONNX（後で）/ autonomy の direct 経路（classical・navigator）/ int8・モデル変更

**記録**: `record_run.sh --name <名前>`。bag ごとに **4 リポジトリのコミット**をメモ（msgs の系列で読めるかが変わる）
- 映像（mp4）は control のカメラノードが出す RTSP (`cam1` 前 / `cam2` 下) から録るので、どのスタックでもそのまま録れる
- 検出 (`/perception_node/detections`) は **AUTO の間しか出ない**（`infer_only_in_auto`、CPU を空けるため）。
  MANUAL 中も欲しいときは `ros2 param set /perception_node infer_only_in_auto false`。ふだんは録った映像に後からかける
  （素の core launch には画像ブリッジも認識も無い）。風船の映像は `record_vision:=true` で起動して `record_run.sh --vision`

**dev-0921 との差**（exp に足してあるもの）: control = hold_yaw / B-20（`servo_max_angular_velocity` 4.0）/
disarm 中の logic 初期化、msgs = `AttitudeTarget.hold_yaw`、ui = R1 トグル。core は dev-0921 のまま。
⚠ dev-0921 の yaml は `servo_max_angular_velocity: 0.0`（推力が出ない値）。向こうの実機で推力が出ているなら
Pi 側で値を変えているはずなので、その値と 4.0 を突き合わせること

---

## 2026-10-01: control 経路で回すときの組み合わせ

**4 リポジトリを揃えないと黙って推力 0 になる。** 1 つでも main 系が混ざると成立しない。

| リポジトリ | ブランチ | 無いと何が起きるか |
|---|---|---|
| `sinsei_umiusi_msgs` | `feat/hold-yaw`（dev-0921 + hold_yaw） | control / core がビルドできない |
| `sinsei_UMIUSI_control` | `exp/20261001-control` | dev-0921 素のままだと **B-20 で推力 0** |
| `sinsei_UMIUSI_core` | `dev-0921` | main のままだと **誰も `/cmd/attitude_target` を出さない** → 目標 quaternion が 0 で fb が出力しない |
| `sinsei_UMIUSI_ui` | `feat/hold-yaw`（dev-0921 + R1 トグル） | main のままだと AttitudeTarget を送らない |

`exp/20261001-control` の中身: dev-0921 + **hold_yaw** + **B-20 修正**（`servo_max_angular_velocity` 4.0）+
**disarm 中は logic を初期化し続ける**。build OK / 205 tests pass。
**実機での起動は未確認**（mock ハードウェアが無いので手元で立ち上げられない）。

- **autonomy の `classical_attitude` を同時に上げないこと。** `/cmd/direct` に publisher が居ると
  control の logic が丸ごと迂回される（B-12）。`preflight.py` が検出する
- **操縦**: 左スティック縦 = 前後 / 横 = ヨーレート（最大 1.0 rad/s）/ 右スティック = roll・pitch（最大 0.3 rad）/
  左右キー = 横移動 ±0.5 / L2・R2 = 上下 ±0.3 / **R1 = 方位保持 ON/OFF（通知が出る）**
- 方位保持中の左スティック横は**保持方位を回す**。0.1 s 入力が途切れると core が目標をクリアし、保持も外れる
- **この組み合わせの bag は `dev-0921` 系の msgs で録られる。** main 系の msgs では読めない（逆も同じ）
- **UI だけで回せる流れ**: Power On → **MANUAL**（core が 4 基を runnable にし、manual_target_generator を
  起動）→ ゲームパッド。**STANDBY = disarm**。**AUTO** は今は空の Target しか出さない仮実装なので、
  モード遷移の確認に安全に使える（姿勢は水平・ヨーレート 0 を保つはず）
- **競技の自律 (AUTO)**: `ros2 launch umiusi_autonomy core_autonomy.launch.py` で core の代わりに起動する
  （core の空の auto_target_generator の代わりに、FSM 入りの同名ノードが上がる）。UI で **AUTO** にすると
  FSM が `/cmd/target`（正規化）と `/cmd/attitude_target`（ヨーレート、方位保持なし）を出す。**STANDBY で止まる**
  - **前進の符号は UI と同じフィールド**。先に MANUAL でスティック前 → 前進を確認すること。逆なら
    `surge_sign:=-1`（auto_target_generator のパラメータ）
  - 風船が見えない間は SEARCH: その場で 0.5 rad/s 旋回 + 上下に小さく揺れる。これが出れば配線は通っている
  - surge 0.22〜0.34 は duty 0.22〜0.34 になる（cap 0.5）。autonomy の旧経路 (cap 0.25) より強い
  - **DEBUG モードは使わない**（`/debug_thruster_output` が無く core が待ち続ける）
  - 実機の perception（カメラ → 検出）は未確認。検出が出なければ SEARCH のままになる
- **シェルが要るもの**: `record_run.sh`（録画）/ `preflight.py` / `thruster_cmd.py`（duty の階段・サーボ 90° ステップ）。
  `thruster_cmd.py` は `/cmd/direct` を出すので、**走っている間は control の logic が迂回される**（B-12。この試験ではそれで正しい）。
  **STANDBY で使う**こと（MANUAL のゲームパッド指令と混ぜない）
- **角度は全部 rad**（B-22）。`thruster_cmd.py` / `thrust_sign_check.py` の `--angle` も rad。
  deg のつもりの値（|x| > pi/2）は弾く

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

## 安全側に倒れたとき — **故障ではないが、原因は必ず追う**

姿勢制御器が自分で出力を絞る場合が 3 つある（known_issues B-19）。**どれも「機体が壊れた」
ではなく「入力が来ていない」の合図**なので、慌てず、ただし**原因を潰すまで次の run に行かない**。

| 見えるもの | 何が起きているか | どうする |
|---|---|---|
| **並進だけ急に止まる**（姿勢は保たれている） | 速度指令が 1 秒来ていない（デッドマン） | UI / テザー / FSM 側を疑う。指令が戻れば**自動で復帰** |
| **出力が全部 0 になる** + ログに `IMU が ... 途切れている` | IMU の断。姿勢制御が成立しないので止めている | 正浮力なので**浮上側に外れる**。IMU が戻れば自動で復帰 |
| `制御の計算に失敗 ... この周期は 0 を出す [n/10]` | 1 周期ぶん捨てた | 1〜2 回なら続行。**10 周期続くと自動で disarm する** |

切りたいときは `ros2 param set /classical_attitude vel_timeout 0`（`imu_timeout` も同様）。
**切ったまま出艇しないこと。** `tools/preflight.py` が切れていないかを見る。

## 次に実機を回すとき、**ついでに測っておくもの**

sim 側の評価が、ここの実測値に依存している。**姿勢制御を 3 分回した bag が 1 本あれば足りる**。

| 測るもの | どうやって | なぜ |
|---|---|---|
| **自由減衰**（最優先・所要 1 分） | **disarm したまま**、機体を手で 15〜20° 傾けて**放す**。roll と pitch を別々に。放してから 30 秒 bag を録る | 復元の強さ `d` が**制御トルク抜きで直接出る**（周期 T から `d=(2π/T)²`）。いまの `d` は姿勢制御を回している最中の回帰なので、制御と復元を分離した値。sim の復元は **roll 2.3 倍・pitch 4 倍強すぎ**と出ており、ここが固まらないと学習も評価も前提が狂う |
| **esc の符号反転の頻度** | `python3 tools/reversal_check.py <bag>` | sim では 1.9 回/秒、9/13 の実機では 0.02〜0.60 回/秒。**3 倍以上ずれている**。9/13 の bag は特異点回避の修正より前なので、**新しいアロケータでの実測が要る**。ここが 0.6/秒側なら、死に時間による被害はかなり縮む |
| **深度指令の向き** | 深度指令を一方向に 10 秒 | **垂直の符号を実機で確かめられる唯一の観測点**（`/state/pressure` が無いので目視） |
| **teleop の前後** | スティックを前に倒す | `is_forward` を全 true に戻した後の確認 |

> **自由減衰の合否判定**: **roll と pitch で卓越周期が違うこと。** 同じ周期が出たら、
> それは機体の固有振動ではなく**テザーや波の外力**を見ている（9/13 の bag の静止区間は
> すべてこれで、roll と pitch が同じ峰を共有していた — 慣性も復元も違う 2 軸が偶然
> 一致することは無い）。目安は **roll 約 5 s / pitch 約 9 s**（いまの推定値から）。
> 振幅が減っていくことも確かめる（減らないなら外力で揺すられている）。

## 効きが足りない / 行き過ぎるときのつまみ

**どれも既定は据え置き**（実機で 1 回も確かめていないため）。**現場で試して、効いたら報告する**。

| 症状 | 打つもの | 根拠 |
|---|---|---|
| 旋回の追従が甘い（目標が先に行きすぎる） | `ros2 param set /classical_attitude cmd_target_yaw_lead_max 0.35` | 追従誤差 p95 が 29.2° → 20.1°、旋回レートは 5% しか落ちない。**180° の罠への余裕も増える** |
| 方位を止めたときに行き過ぎる / 揺り戻す | `ros2 param set /classical_attitude kd 0.5` | **起動の死に時間があると kd の最適が 0.35 → 0.5 に上がる**（偽機体と MuJoCo の 2 つで一致）。**迷ったら上げる。下げてはいけない** — kd=0 だと行き過ぎが 33° まで出る |
| 3 基運用で風船に寄るのが遅い | `ros2 param set /classical_attitude max_duty 0.4` | 3 基だと duty が全域で上限に張り付くので、**cap がそのまま前進速度**（0.25→0.11 / 0.4→0.20 m/s） |
| **姿勢がすぐ飽和する**（duty が上限に張り付いて戻らない） | 同上 `max_duty 0.4` | **浮力トリムだけで推力を食っている**。全推力に占める割合は 4 基で 15.6%(cap 0.25) / 10.8%(0.3) / 6.1%(0.4)、**3 基なら 20.8% / 14.5% / 8.1%**。cap 0.25 の 3 基運用は、出せる推力の 1/5 を浮いているだけで使う |

> `kd` / `kp` は**負でバンドルの値**を使う（`k_v_vert` と同じ規約）。0 以上を入れたときだけ上書き。
> 数字の出どころは `docs/performance_tuning.md`。**kd の差 0.35 vs 0.5 は 4% で、死に時間に
> よる 8 倍悪化の前では誤差**なので、まず上 2 つを試すこと。

## 直ちに止めるもの

- `water_leaked` が `true` になった
- ESC の電圧が **20 V を下回った**
- 異音・異臭・発熱
- 1 基だけ極端に挙動が違う
- **判断に迷った**

そのあと運用担当に連絡する。**止め方は各節の `arm` サービス。**

---

## スラスタが 1 基死んでいるとき

**古典なら動かせる**（残り 3 基でも 6 自由度の権限は残る。rank 6・条件数 5.34→8.05）。
**RL は使えない**（方策が 4 基前提）。**2 基以上死んだら諦める。**

### いまの状態（2026-09-20 更新）

**実機では縮退が働いていた。ただし設定は git に無い。** 9/13 のプール bag に
`classical_attitude: live_thrusters=[False,True,True,True] (control から取得 (esc_disabled))`
が残っており、`/cmd/direct` の lf の duty は全区間 0 だった。

> **しかし control の PR #317 (`codex/left-front-thruster-failure`) は CLOSED で、
> ブランチも削除されている。** `origin/main` の `params/controllers.yaml` は
> **4 基とも `esc_disabled: false` / `is_forward: true`** で、`disabled_thruster` も無い。
> つまり **9/13 に動いていた設定は Pi の上だけに在る未コミットの手編集**。
> **Pi を再クローンすると消える。** 逆に Pi で `git pull` すると、手編集が消えるか衝突する。
> **出艇前に `tools/preflight.py` で「control が実際に何と答えるか」を必ず見ること。**

以下は**その設定が入っていない機体で動かすとき**の話。autonomy は既定で control の
`esc_disabled` を読むので、入っていなければ「全基生きている」と判断して lf に配分し続ける。
どちらかを選ぶ:

**A. control のブランチをマージする**（推奨。設定が 1 箇所で済む）

```yaml
attitude_controller:      disabled_thruster: "lf"   # control の ff 経路用
thruster_controller_lf:   esc_disabled: true        # direct 経路にも効く
```

これだけで autonomy は起動時に読んで自動で外す。

**B. マージ前に動かすなら autonomy 側で明示する**

```bash
ros2 launch umiusi_autonomy scenario.launch.py   # 起動してから
ros2 param set /classical_attitude live_thrusters_source param
ros2 param set /classical_attitude live_thrusters '[false,true,true,true]'   # lf,lb,rb,rf
ros2 param set /classical_attitude max_duty 0.4   # 余裕が無くなるので上げる
```

ログに `**lf を死亡扱いにした。**` が出れば効いている。**出なければ効いていない。**

### 効き目（実測）

| | 外さない | 外す |
|---|---:|---:|
| yaw トルク指令の実現誤差 | **240%** | 0% |
| 前進指令の実現誤差 | 57% | 0.2% |
| **水平ドリフト** | **1.36 m** | **0.02 m** |

**効くのは主に並進。** 姿勢は浮力の復元でそれなりに保ててしまうので、
**姿勢だけ見ていると「外さなくても動いている」と誤読する。**
競技は風船へ近づく動作なので、水平 1.36 m のずれは致命的。

> **外さないと「死んだ基に配分し続ける」**状態になり、解いた力と実際に出る力が食い違って
> 残り 3 基が誤った前提で釣り合いを取る。**control 側だけ直しても direct 経路は直らない。**

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

**`ModuleNotFoundError: No module named 'scipy'` が出たら scipy だけ入れる**:

```bash
pip install --user scipy
```

（制御則そのものは numpy しか使わないが、`umiusi_perception` の `__init__` が
検出器を経由して **scipy** を引く。**torch と cv2 は遅延 import なので要らない** —
`--no-deps` を付けるのは torch の解決で時間を食わないためで、実測で確認済み）

通らなければ `export PYTHONPATH=~/umiusi_sim/packages/perception/src:$PYTHONPATH`
（**その窓から launch すること**）。詳細は `robot_setup.md`。

**ビルドし直さない。** どうしても必要なら `rm -rf build/umiusi_autonomy install/umiusi_autonomy`
してから `colcon build --packages-select umiusi_autonomy --symlink-install`。

---

## 1. 推力の符号を確定させる ← **最優先。これが通らないと以降は全部無意味**

> **2026-09-13: 原因を特定してバンドルを修正した**（`thrust_axes` の符号規約が control と
> 逆だった。known_issues B-14）。**ハードは触っていない。** 偽機体では直ったが、
> **実機での確認はまだ** — 下の手順で必ず確かめること。
>
> lf は無反応だった（故障と一致）。**`live_thrusters` の設定も忘れずに**（上の節）。

2026-09-12 のプール実験で、**指令したモーメントと機体の動きが 3 軸とも逆**だった。
arm すると振れが 2〜3 倍に悪化し、yaw は目標から 180° 離れたところで安定していた。
どの基が反転しているかは**あの bag からは分離できない**（4 基が一斉に動いていたため）。
**1 基ずつ測る。4 基すべて。**

### 1-0. まず**全基を規定姿勢に置いて、4 基の相対関係を見る**

1 基ずつ見る前に、**全部同じ指令を入れて揃っているか**を一度に見る。
サーボが 1 基だけ違う向きに寝ている、という類はここで出る。

```bash
python3 tools/thruster_cmd.py pose                  # 全基 servo 0° / duty 0.1 を 20 s 保持
python3 tools/thruster_cmd.py pose --angle 0.785    # 全基 45° (rad で指定) に寝かせて保持
python3 tools/thruster_cmd.py pose --angle -1.571   # 全基 真下向き (-90°)
```

**指令を出す前に「何が起きるはずか」を表示する**ので、それと見比べる:

| 指令 | 起きるはず |
|---|---|
| `pose`（servo 0°・全基 +0.1） | **上から見て右回り**（合力ちょうど 0 の純粋な旋回） |
| `pose --angle 0.785` | 上昇・右回り・機首が上がる |
| `pose --angle -1.571` | 下降・機首が下がる（旋回なし） |

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

### 直しかた — **どこを直すかで意味が違う**

符号を変えられる場所は 4 つあり、**効く範囲が違う**。取り違えると別のところが逆になる。

| 直す場所 | 何が反転するか | 効く経路 | こういうとき |
|---|---|---|---|
| **`thrust_axes`**（バンドル） | **水平成分だけ**（垂直は `_Y_UP` で別管理） | autonomy の direct のみ | **モデルの規約が実機と逆**のとき。2026-09-13 はこれだった |
| **`thrust_sign`**（autonomy param） | **推力ベクトル全体**（水平も垂直も）。基ごと | autonomy の direct のみ | 現場で当たりを探す**一時対処**。再起動で戻る |
| **`is_forward`**（control の yaml） | その基の duty の符号 | control の FF + **autonomy が起動時に読む** | **実機の極性**が逆（ペラ・モータ相・ESC 設定） |
| **`servo_sign`**（autonomy param） | サーボの回転センスだけ | autonomy の direct のみ | サーボの正方向が逆 |

### 選びかた

```
thrust_sign_check.py の結果
├─ 水平 (0°) と垂直 (60°) の両方が反転
│    → 推力ベクトルごと反転 = **実機の極性**。control の is_forward を直す
├─ 水平 (0°) だけ反転
│    → **モデルの規約**が逆。バンドルの thrust_axes を直す
├─ 垂直 (60°) だけ反転
│    → サーボの正方向。servo_sign を直す
└─ 4 基とも同じ向きに反転 かつ control の FF 経路は正常に動いていた
     → **モデルが逆。ハードを触らない**（触ると FF と UI テレオペが逆になる）
```

> **60° の判定は信用度が低い。** 予測トルクに占める yaw の割合が 0° では 100%、
> 60° では 29% しかなく、roll/pitch は浮力の復元と戦うので Δω がトルクに比例しない。
> **0° の結果を主に、60° は参考に。** 同じ理由で**配線の入れ替わり検出も 60° 依存**なので
> 鵜呑みにしない。

### 2026-09-13 に実際にやった直しかた

実測で **lb/rb/rf の 3 基とも 0° で反転・lf は無反応（故障）**。そして:

| | 全基の水平出力を正にしたときの回転 |
|---|---|
| control の FF（コメントに明記） | 反時計回り |
| **実機の実測** | **反時計回り** |
| バンドル | **時計回り** ← これだけ逆 |

→ **機体と control が一致し、モデルだけが逆**。よって `thrust_axes` を反転させた。
**ハードは触っていない。**

> **2026-09-20: この判断は 9/13 のプール bag で裏が取れた（known_issues B-14 追記）。**
> `tools/thrust_sign_from_bag.py` で軸ごとに測ると、当日の現場構成（旧バンドル +
> `is_forward` 全 false）は **yaw は正しく、roll/pitch が反転**していた。つまり反転して
> いたのは**水平だけ**で、`thrust_axes` を直したのが正解。
>
> **現場が入れた `is_forward` 全 false は元に戻すこと（全 true）。** あれは yaw を直す
> ための応急処置で、引き換えに **roll / pitch / 深度が反転**していた。`main` のバンドルの
> まま false にしておくと **6 自由度すべてが反転する。**
>
> 次からは専用の実験を回さなくても、**姿勢制御を回した普通の bag があれば符号は測れる**
> (`python3 tools/thrust_sign_from_bag.py <bag>`)。

```bash
# 現場で試すだけなら (再起動すると戻る)
ros2 param set /classical_attitude thrust_sign '[1.0,-1.0,1.0,-1.0]'   # lf,lb,rb,rf
```

> **バンドルを sim から再エクスポートするとこの修正は巻き戻る**
> （`configs/umiusi.yaml` は旧規約のまま）。`export_classical.py` を流す前に確認すること。

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

**`cmd_target_yaw_mode=rate — orientation.z を旋回レートとして積む` がログに出れば効いている。
出なければ効いていない。** 旋回だけができないときは、まずここを疑う（符号ではない・B-17）。
旋回が遅い / 速いときは `ros2 param set /classical_attitude cmd_target_yaw_rate_scale 2.5`
（既定 1.5 rad/s。左スティック −0.2 なら 0.3 rad/s = 17°/s）。

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

## 今日ほしいデータ（2026-10-01 更新）

> **符号は 9/13 の bag で決着済み**（B-14 追記）。この節の旧版は「符号確認が最重要」だったが、
> もう済んでいる。**今日は `is_forward` が全 true であることを preflight で確かめるだけ。**

| | なぜ |
|---|---|
| **定常 duty の階段**（各 duty を数秒保持）| **`thrust_curve_exp` が未較正のまま残っている唯一の理由がこれ。** テレオペの bag では rpm が 3.3〜12 Hz しか更新されず、46 Hz の duty とペアにならないので回帰が壊れる（R² が負）。保持すればレートは問題にならない。`tools/thruster_cmd.py steady` → `tools/duty_rpm_fit.py` |
| **自由減衰 1 分**（disarm で 15〜20° 傾けて放す）| sim の浮力復元が強すぎる問題（offset 実測 2〜4 mm vs sim 10 mm）を詰める材料。合否 = **2 軸で周期が違う / 振幅が減衰する** |
| **バラスト調整の前後で「指令 0 のまま」の bag**（各 30 秒）| **B-21 の確認。** いま指令 0 でも 1 基 duty 中央 0.17〜0.21（cap の 52〜68%）を浮力トリムが食っている。調整後に `tools/idle_thrust_check.py` で `Σ|duty| ≈ 0` になるか |
| **推力を出しながらの IMU**（長めに 1 本）| **yaw 跳躍の発生率が「推力あり」で一度も測れていない。** 169° の観測は陸上・スラスタ停止中の n=1。水中 61 分では 0 件。`tools/imu_jump_axis.py` |
| 姿勢制御の bag | 発振が消えたかの判定。`run_compare.py` にかける |
| **並進しながらの cam2 映像** | 速度推定の素材。sway 問題はこれ待ち |
| **風船の実写** | 広いプールのものが 0 枚。赤黄青を同じ画角に / 距離を変える / **見上げ** / **風船なしも同量** |

> **bag と一緒に `sinsei_umiusi_msgs` のコミットもメモすること。**
> `dev-0921` 系の msgs は `ThrusterState.angle` を `commanded_angle` に改名しているので、
> **録ったときと違う系列の msgs で読むと `RMWError: failed to deserialize` で落ちる。**

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
