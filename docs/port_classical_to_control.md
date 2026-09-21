# 古典制御を `sinsei_UMIUSI_control` へ移すための TODO

いま古典制御は **autonomy 側の Python** にある:

```
umiusi_perception.classical        制御則・観測器・アロケータ (sim と実機で同じコード)
  ClassicalController              姿勢 PID + heave + cap の LPF
  VelocityObserver                 指令から速度を推定する (深度センサが無いため)
  GeneralAllocator                 幾何の擬似逆行列 + 零空間の特異点回避 + 欠損対応
  PlantContract                    バンドル JSON の契約値
umiusi_autonomy/classical_attitude_node.py   上を呼ぶだけの薄い ROS ラッパ
```

control へ移すと `logic::attitude::Classical` になり、`control_mode:=classical` で選ぶ形
（既存の `ff` / `rl` と同じ並び）。`AttitudeController::Logic` の入出力は**すでに必要な形**を
している（in: `target_orientation` / `target_velocity` / `imu_*` / `esc_rpms`、
out: `esc_thrusts[4]` / `servo_angles[4]`）。Eigen も既に依存に入っている。

---

## 行き先 — **autonomy は最終的に navigator だけになる**

（ユーザー方針 2026-09-22）移植が終わったあとの姿:

```
control (C++)                          autonomy (Python)
  logic::attitude::Classical  <--- AttitudeTarget ---  navigator_node  (FSM)
  ThrusterLimits / esc_disabled                        perception_node (検出器)
  is_forward / servo_sign                              camera_bridge_node
```

- **autonomy に残るのは「認識して、どこへ行きたいかを出す」側だけ。** 姿勢の安定化・配分・
  スラスタへの出力は全部 control。`classical_attitude_node` は役目を終えて消える。
- **`/cmd/direct` を publish するノードが autonomy から無くなる。** これで B-12 の迂回が
  構造的に起きなくなる（「publisher が居ると logic ごとスキップ」の publisher が居ない）。
- `navigator_node` は既に `command_mode:=setpoint` で `AttitudeTarget` を出すだけの形に
  なっている。**移植後はこれが唯一の経路**で、`direct` / `target` モードは消せる
  （どちらも配分を自前に持ち姿勢の安定化が無い、という今の問題がそこで終わる）。
- **消えるもの**: autonomy 側の `thrust_sign` / `servo_sign` / `live_thrusters` /
  `max_duty` / スルーレート / 断の検出（B-19）。**全部 control 側の 1 箇所になる。**
  それぞれ control に等価物を作ってから消すこと（§5・§6 がその表）。

移植の順序としては、**navigator の setpoint 経路を壊さないことが唯一の制約**になる。
`AttitudeTarget` の規約（下の §0 の 2 番目）さえ固定すれば、control 側の実装は
差し替えとして進められる。

---

## なぜ移すのか — 効く順

1. **`/cmd/direct` の迂回が無くなる。** いまは autonomy が `/cmd/direct` に publish するため、
   control の logic が**丸ごとスキップ**される（known_issues B-12）。その結果
   `max_duty` もスルーレート制限も `is_forward` も control 側では効かず、**同じ設定を
   2 箇所に持つ**羽目になっている（B-14 の `thrust_sign` はそのための導線）。
   移せばこの分岐が消える。**これが最大の利得。**
2. 安全系（`esc_disabled` / `max_duty` / スルーレート）が 1 箇所に揃う。
3. Pi 上の Python プロセスが 1 つ減る（制御周期のジッタが減る）。

## 何を失うか — **先に決めるべき唯一の論点**

いま `umiusi_perception.classical` は **sim と実機で同じコードが動いている。**
記録に残る「sim では正しいが実機で変」4 件はすべて軸や系の取り違えで、
**同じコードにしたことが再発防止になっている。**

C++ に書き直すと**実装が 2 本**になり、片方だけ直る事故が戻ってくる。

> **これを埋める手当て無しに移すのは、B-14 の再発を買うのと同じ。**

手当ての候補（どれかを必ず採ること）:

- **(A) ゴールデンベクタで言語間パリティを縛る**（推奨・低コスト）
  Python 側で「入力 → 出力」の組を N 点書き出し、C++ のテストがそれを読んで一致を検査する。
  バンドル JSON を入力にすれば契約値も一緒に縛れる。**許容差は 1e-9 で始める。**
- (B) C++ を正として Python は薄いバインディング（pybind11）にする。
  sim/学習が C++ ビルドに依存するようになるので、sim 側の開発コストが上がる。
- (C) 移さない。`/cmd/direct` の迂回は B-12 の運用でしのぐ（今の状態）。

---

## TODO（順番どおりにやる）

### 0. 決める

- [ ] **上の (A)/(B)/(C) を決める。** (A) 前提で以下を書いてある
- [ ] **`Target.orientation` の規約を 1 本に決める**（known_issues の open question）。
      いま `rl.hpp` = 回転ベクトル [rad] / `feed_forward.hpp` = 無次元トルク /
      autonomy の navigator = 正規化指令、で **3 通りに割れている**。
      Classical logic は `target_orientation` を受けるので、**ここを決めないと移せない**
- [ ] 移行中の二重駆動を防ぐ手順を決める（`control_mode:=classical` にしたら
      autonomy の `classical_attitude` は上げない。`preflight.py` の B-12 検査が効く）

### 1. 契約値の受け渡し

- [ ] `classical_bundle.json` を control から読む口を作る（`bundle_path` パラメータ）。
      **JSON パーサの依存を増やさない**なら、必要な値だけ yaml パラメータへ写す案もあるが、
      **`thrust_axes` の符号が 2 箇所に分かれる**ので推奨しない（B-14 の再発）
- [ ] 読んだ契約値を起動ログに出す（`cap_ref` / `thrust_axes` の yaw 符号 / 未較正フラグ）。
      `tools/preflight.py` が今 autonomy に対してやっている検品を control に移す

### 2. 制御則の移植（`logic::attitude::Classical`）

- [ ] 姿勢 PID（roll/pitch/yaw）— `ClassicalController.wrench`
- [ ] **heave**（速度指令として受ける。`k_v_vert` 既定 0 = 前進項のみ）
- [ ] cap の LPF（`f_max_total(cap)` は**フィルタ後の cap** で評価する。生の cap を使うと
      スケールが `(cap_a/cap_b)^exp` だけ狂う）
- [ ] `hold_yaw`（false のとき yaw の姿勢誤差だけ落とす。**gyro は落とさない**）
- [ ] REP-103 ⇄ CAD の変換は**関数 1 本に閉じる**（Python 側は
      `rep103_from_cad` / `cad_wrench_from_modes`。8 箇所に散らばって事故った履歴がある）

### 3. 観測器の移植（`VelocityObserver`）

- [ ] 指令からの速度推定。**入力は「レート制限後に実際に出した指令」**。
      制限前を渡すと出していない指令で積分する
- [ ] **深度センサが載ったらここは差し替える。** `/state/pressure` が実機に来ていない
      （2026-09-13 の bag に無い）ので、いまは鉛直も推定値。載った時点で
      `k_v_vert` は実測の微分で閉じる

### 4. アロケータの移植（`GeneralAllocator`）— **いちばん重い**

- [ ] 幾何行列 A の構築（水平列 = `thrust_axes`、垂直列 = `_Y_UP`）と擬似逆行列。Eigen で可
- [ ] **欠損対応** `set_live()`。1 基落として解き直す（rank 6 維持・条件数 5.34 → 8.05）。
      **2 基以上は拒否する**（6 自由度の権限が無くなり、黙って姿勢が崩れる）
- [ ] **零空間の特異点回避。** ここが移植の山。
      - 零空間の正規直交基底（SVD）
      - 特異点までの距離とサーボ移動量の**両方**を含むコスト
      - **`into` は折返し「後」の角度で採点する**（折返し前で採点すると回避が一度も
        動かない。sim 側で 2026-09-21 に見つかった実バグ）
      - ホールド中の最小ノルム解は厳密に鉛直で、零空間 2 方向はどちらも h を
        符号交代でしか作れない ⇒ **どう逃げても半数が 90° を越える**。この性質を
        テストに固定する
- [ ] servo のスルーレート制限（既定 250 deg/s）。特異点回避はこの上限を前提に調整してある

### 5. 出口（符号・欠損・上限）

- [ ] **符号は出口だけで掛ける。** 上流で掛けると観測器が「出していない指令」で積分する
- [ ] 極性の正は **control の `is_forward` 1 箇所**（移行後は autonomy 側の `thrust_sign` を
      廃止できる。B-14 の二重管理が消える）
- [ ] `esc_disabled` → アロケータの `live` へ。**いまは control と autonomy の 2 経路**で、
      preflight が一致を見ている。移行後は 1 経路
- [ ] `max_duty` / スルーレート制限が**実際に効く**ことを確認（B-12 が消えるので効くはず）

### 6. 断の検出（B-19 を移す）

- [ ] `imu_timeout` — IMU が途切れたら出力 0（disarm はしない。正浮力なので 0 は浮上側）
- [ ] `vel_timeout` — 速度指令が途切れたら**並進だけ** 0、姿勢目標は保持
- [ ] どちらも**既定で有効**にする（既定 0 だといざというとき入っていない）

### 7. テスト

- [ ] **(A) のゴールデンベクタ**: Python が書き出した入出力を C++ のテストが読んで一致検査
- [ ] 欠損（lf 死亡）で 6 自由度が出ることの単体テスト
- [ ] `control_mode:=classical` の起動テスト（`ff` / `rl` と同じ並びで落ちないこと）
- [ ] **`ros2 param set` で変えられると書いた param が、本当にコールバックで受けているか**の
      パリティテスト（autonomy 側の `test_cmd_target_yaw.py` と同じ形。B-17 の再発防止）

### 8. 撤去

- [ ] **autonomy の `classical_attitude_node` を消す**（行き先の図のとおり。残すと
      `/cmd/direct` の publisher が居る構成が復活し、B-12 が戻る）
- [ ] navigator の `command_mode` から `direct` / `target` を消し、`setpoint` 一本にする
- [ ] `docs/` の経路の説明を 1 本に直す（`field_card` / `scenario_run` / `teleop_gamepad`）
- [ ] `tools/preflight.py` の検査対象を control 側へ向け直す

---

## 移行後に消えるもの（利得の確認用）

| いまの回避策 | 消えるか |
|---|---|
| known_issues B-12（`/cmd/direct` が control の logic を丸ごと飛ばす） | **消える** |
| B-14 の `thrust_sign` 二重管理（autonomy が control の `is_forward` を読みに行く） | **消える** |
| B-16 の「縮退の経路が 2 つある」 | **消える** |
| B-19 の断検出 | control 側で作り直す（移す） |
| B-17（param が実行中に効かない） | **形が同じなので C++ 側でも作り込む必要がある** |

## やらないこと

- **RL 経路の移植は含めない。** `logic::attitude::Rl` は既にあり、方策の再学習は
  競技後（当日の映像を集めてから）。ここでは触らない
- 学習済み方策との互換維持。2026-09-21 にユーザー判断で**捨てる**方針
