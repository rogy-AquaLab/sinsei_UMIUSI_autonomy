# autonomy と control(`dev-0921`) の差分、と決めなければいけないこと

**前提**（ユーザー方針 2026-09-29）:
- 仕様は **control 側に統一**する。autonomy が合わせに行く
- autonomy は最終的に**無くなる**（姿勢制御は control、FSM/認識は core へ）
- lf は**直った**ので 4 基運用。欠損対応は当面の律速ではない

この文書は **「どちらが正しいか」ではなく「決めないと進めないもの」の一覧**。
実装の良し悪しの評価は `port_classical_to_control.md`、実機で踏んだ事実は `known_issues.md`。

---

## 1. インタフェース（msgs）

| | autonomy が今使っているもの | control `dev-0921` |
|---|---|---|
| **`Target`** | `velocity` + **`orientation`** | `velocity` のみ（**`orientation` 削除**） |
| **姿勢目標** | `umiusi_rl_control_msgs/AttitudeTarget`<br>`orientation`（**yaw 込みの絶対姿勢**）+ `velocity` + `type_mask` | `sinsei_umiusi_msgs/AttitudeTarget`<br>`attitude`（**roll/pitch のみ・yaw は 0**）+ **`yaw_rate`** |

**壊れる箇所**（`Target.orientation` を読んでいる/書いている）:
- `classical_attitude_node._on_cmd_target` — **UI テレオペが姿勢制御に乗る経路**（B-17 で直した所）
- `navigator_node._publish_target` — `command_mode:=target`
- UI の `gamepadPublisher.ts` と core の `manual_target_generator`

---

## 2. 制御則

| | autonomy（`umiusi_perception.classical`） | control `dev-0921` |
|---|---|---|
| roll / pitch | 絶対姿勢 PID | 絶対姿勢 PID（reduced-attitude、body-up を合わせる） |
| **yaw** | **絶対方位**（`hold_yaw`）。FSM のレートを積分して作る | **レート制御**。目標 quat の yaw は無視 |
| 積分項 | `ki` 既定 0 | `ki_roll` / `ki_pitch` 既定 0 + アンチワインドアップ |
| **並進** | **閉ループ**。速度観測器 `VelocityObserver` の推定値と `k_v` で追う | **開ループ**。`target_velocity` を配分へ直入れ |
| **浮力トリム** | **あり**（`net_buoy_up` から下向きの一定トリム） | **無い** |
| heave | `k_v_vert`（既定 0 = 前進項のみ） | `velocity.z` が配分へ入るだけ |
| cap | `cap_norm` / `cap_tau` の LPF | 無し（`ThrusterLimits` が出口でクランプ） |

> **浮力トリムが無いと機体は浮きます。** 正浮力 +1.17 N は cap 0.25 の全推力の 15.6%。
> いま autonomy はこれを常時打ち消しています。

---

## 3. 配分（アロケーション）

| | autonomy `GeneralAllocator` | control `mixer.hpp` |
|---|---|---|
| 解き方 | **幾何の擬似逆行列**（`thrust_axes` と `_Y_UP` から A を組む） | **固定行列**（`feed_forward` と同じ `a`） |
| 欠損対応 | **`set_live()` で 1 基落として解き直す**（rank 6 維持・条件数 5.34→8.05） | **無い**。`esc_disabled` を読まない |
| 特異点回避 | **零空間のコストで回避**（サーボ移動量 + 特異点距離） | 無い。±90° 近傍で 180° 回さない処理はある |
| **サーボの実角** | **使わない**（指令角前提） | **推定角を使い、現在の推力軸へ要求を射影する** |
| サーボのレート制限 | ノード側で slew | mixer が `max_angular_velocity * duration` で制限 |

> **サーボ推定角の射影は control 側が優れています。** 「サーボは追従中、ESC は到達後を前提」
> という不整合が起きません。**autonomy にはこれがありません。**
> 逆に**欠損対応と特異点回避は control に無い**（9/13 実測で、欠損を伝えないと yaw 実現誤差 240%）。

---

## 4. 安全・運用

| | autonomy | control `dev-0921` |
|---|---|---|
| 入力の断 | **`imu_timeout` / `vel_timeout` 既定 1.0 s**（B-19） | 無い。ただし**サーボ推定角が無効なら出力 0** |
| 例外 | `_tick` で捕まえ、10 周期続けば disarm | — |
| 現場のつまみ | `thrust_sign` / `servo_sign` / `live_thrusters` / `max_duty` / `kd` / `kp` / `cmd_target_*` を実行中に変更可 | `is_forward` / `esc_disabled`（元からある） |
| 出艇前の検品 | `tools/preflight.py` | — |
| 極性の正 | control の `is_forward` を起動時に読む | `is_forward` が正 |
| **`servo_sign`** | **autonomy にある** | **`fix/deploy-hardening-2026-09` で control にも入る** |

> ⚠ **`servo_sign` が両方に存在すると、両方 −1 で打ち消し合います**（既定 1.0 同士なので今は無害）。
> `thrust_sign` / `is_forward` で踏んだ二重管理と同じ形。

---

## 5. 決めなければいけないこと

### D-1. `AttitudeTarget` の yaw — レートのみか、保持を残すか 🔴

`dev-0921` は**レートのみ**。autonomy は**絶対方位**（`yaw_lead_max` で 180° の罠を防いでいる）。

- **レートのみにすると**: 180° の罠（B-14）が**構造的に起きなくなる**（積む対象が無い）。
  代わりに「手を放すと方位を保つ」が消え、**水流で流されても気付けない**
- **保持を残すなら**: IMU の yaw は**磁気基準**（BNO055）で、**BLDC 4 基が直下で回っている**。
  A-1 に「0.5 秒で −3° → −170° → −4°」「150 秒に 1 回、姿勢基準が飛ぶ」と記録がある
- **折衷案**（議論中）: `yaw_rate ≈ 0` が続いたら方位をラッチし、**誤差にクランプと寿命を付ける**。
  クランプを超える誤差は「IMU が飛んだ」とみなしてラッチし直す（追いかけない）

> **関連**: 水流で流されているとき、**カメラ以外に直進を担保する手段**が要る、という論点がある。
> 方位保持はその候補。**保留中。**

**決めること**: ①レートのみ / 保持あり ②保持ありなら クランプ幅・寿命・置き場所（control か core か）

### D-2. `Target.velocity` の単位 🔴

UI は**正規化スティック値**しか出さない（物理単位ではない）。autonomy は `cmd_target_vel_scale`
で m/s に直している。**control が `velocity` を m/s と解釈するのか正規化値と解釈するのか。**

割れると B-17 の再来（設定は入るのに意図と違う速度が出る）。**どちらでもよいが、決めて書くこと。**

### D-3. 浮力トリムを誰が持つか 🔴

autonomy は常時打ち消している（正浮力 +1.17 N = cap 0.25 の全推力の 15.6%）。
`dev-0921` には無い。**このままだと機体は浮きます。**

**決めること**: control の `AttitudeFeedback` に入れるか / 上位（core の FSM）が
`velocity.z` で常時指令するか / 機体側でバラストを調整するか

### D-4. 並進を閉ループにするか

autonomy は速度観測器で閉ループ、control は開ループ。**観測器は深度センサが無いため指令からの
推定**（`/state/pressure` が実機に来ていない）。**開ループのままなら「指令した速度で進む」保証は無い**
が、そもそも観測器も推定値なので**精度の差は小さい**可能性がある。

**決めること**: 開ループで進めるか / 観測器も移植するか（移植は重い）

### D-5. 欠損スラスタ対応をいつやるか

lf が直ったので当面不要。ただし **`dev-0921` の mixer は固定行列なので、次に 1 基死んだら
同じ穴**（yaw 実現誤差 240%）。既存ブランチは**無い**（PR #317 は CLOSED・ブランチ削除済みだが、
**差分は `gh pr diff 317` で回収できる**。ただし `ff` 経路のもので mixer とは別物）。

**決めること**: 競技前にやるか / 後回しか

### D-6. `servo_sign` の正をどちらに置くか

`fix/deploy-hardening-2026-09` を入れると **control にも `servo_sign` ができる**。
autonomy にもある。**両方に −1 を入れると打ち消し合う。**

**決めること**: control 側を正にして autonomy 側を消す（移行方針と整合）でよいか

### D-7. `hold_yaw` / `type_mask` 相当の機能をどうするか

autonomy の `hold_yaw`（yaw の保持だけ切る）と `AttitudeTarget.type_mask`
（姿勢だけ / 速度だけ）は、control 側に**相当物が無い**。D-1 が決まれば `hold_yaw` は
自動的に決まる。`type_mask` は使われていないなら捨ててよい。

---

## 6. 決まれば自動的に片付くもの

| | どの判断で決まるか |
|---|---|
| `classical_attitude_node` の撤去 | 全部（移植完了時） |
| `cmd_target_yaw_mode` / `cmd_target_yaw_rate_scale` / `cmd_target_yaw_lead_max` | **D-1** |
| `cmd_target_vel_scale` | **D-2** |
| `k_v_vert` / `vel_cmd` | **D-3 / D-4** |
| `live_thrusters` / `live_thrusters_source` | **D-5** |
| `thrust_sign` / `thrust_sign_source` / `servo_sign` | **D-6**（極性の正は元から control） |
| `imu_timeout` / `vel_timeout` / `_tick` の例外ガード | **control へ移す**（B-19。判断不要、作業のみ） |
