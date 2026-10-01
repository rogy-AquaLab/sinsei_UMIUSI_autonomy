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
| **yaw** | **絶対方位**（`hold_yaw`）。FSM のレートを積分して作る | **レート制御**。目標 quat の yaw は無視<br>→ **D-1 で `hold_yaw` bool を足して保持も持つ** |
| 積分項 | `ki` 既定 0 | `ki_roll` / `ki_pitch` 既定 0 + アンチワインドアップ |
| **並進** | **閉ループ**。速度観測器 `VelocityObserver` の推定値と `k_v` で追う | **開ループ**。`target_velocity` を配分へ直入れ |
| **浮力トリム** | **あり**（`net_buoy_up` から下向きの一定トリム） | **無い** → **D-3 でこちらが正** |
| heave | `k_v_vert`（既定 0 = 前進項のみ） | `velocity.z` が配分へ入るだけ |
| cap | `cap_norm` / `cap_tau` の LPF | 無し（`ThrusterLimits` が出口でクランプ） |

> **浮力トリムが無いと機体は浮きます。** 正浮力 +1.17 N は cap 0.25 の全推力の 15.6%。
> いま autonomy はこれを常時打ち消しています — **その代償が B-21**（指令 0 でも cap の
> 52〜68% を消費）。**D-3 の決定はバラストで釣り合わせること。**

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
- **保持を残すなら**: IMU の yaw は**磁気基準**（BNO055 を **NDOF モード**で使っている:
  `src/sinsei_umiusi_control/hardware_model/imu_model.cpp:120`）で、**BLDC 4 基が直下で回っている**。
  A-1 に「0.5 秒で −3° → −170° → −4°」「150 秒に 1 回、姿勢基準が飛ぶ」と記録がある

> **測定（2026-09-29）— A-1 の 169° 跳躍は「ヨーだけ」が飛んでいた**（詳細は
> `known_issues.md` A-1 の「跳躍の軸を分解した」節）。回転軸は**ほぼ完全に重力軸まわり**、
> `|q|`=1.00000、同時刻の gyro_z は −0.03 rad/s。
> → **roll/pitch（重力基準）は無傷。壊れるのは yaw（磁気基準）だけ**で、しかも
> 「ドリフト」ではなく **約 180° の跳躍**。
> ⚠ n=1・陸上・スラスタ停止中の観測。**推力を出しているときの発生頻度は未測定。**
- **折衷案**（→ 下の決定へ）: `yaw_rate ≈ 0` が続いたら方位をラッチし、**誤差にクランプと寿命を
  付ける**。クランプを超える誤差は「IMU が飛んだ」とみなしてラッチし直す（追いかけない）

> **関連**: 水流で流されているとき、**カメラ以外に直進を担保する手段**が要る、という論点がある。
> 方位保持はその候補 — **保持を残したので、当面はこれが唯一の手段**。DVL / 流速計 / 光学フローは
> 大会後の話。

#### ✅ 決定（ユーザー 2026-09-29）: **保持を残す。ただし bool + エッジラッチ + 誤差クランプ。**

**自動推定（`yaw_rate ≈ 0` でラッチ）と寿命は入れない。**

```
AttitudeTarget に  bool hold_yaw  を足す（param ではなく msg のフィールド）

hold_yaw = false        … レート制御。dev-0921 の今の挙動そのまま。状態を持たない
false → true のエッジ   … そのときの実測 yaw をラッチして目標にする
hold_yaw = true         … ラッチした方位へ PID。yaw_rate はラッチ値を slew する
                          （latched += yaw_rate * dt）
|誤差| > クランプ       … 「IMU が飛んだ」とみなして現在値へラッチし直す + WARN
disarm                  … ラッチを捨てる
```

**なぜ bool で足りるのか / なぜ bool だけでは足りないのか**

- **bool だけでは目標方位が決まらない。** 「保持しろ」と言われても*どの方位を*保持するのかが要る。
  一番安いのは**エッジでラッチ**すること。つまり **bool はラッチを無くすのではなく、
  「いつラッチするか」の推定を無くす**。消えるのは機構ではなく**推測**
- **自動推定をやめる理由が 3 つ**:
  1. **ノブが 2 つ増える**（`yaw_rate` の閾値と継続時間）。実測の根拠が無い = A-11 の形
  2. **閾値の境目でチャタる。** ラッチし直すたびに方位が少しずつずれる —
     **「流されているのに気付けない」という、保持を残したい理由そのものが壊れる**
  3. **保持中かどうかがワイヤ上で見えない。** bool なら bag に残るし `ros2 topic echo` で分かる
- **クランプは残す。** 実測した唯一の壊れ方（**yaw だけ約 180° 跳ぶ**）への防御がこれ。
  比較 1 行ぶんのコストで、**跳躍が「180° 回れ」という指令に化けるのを止める**。
  クランプが無いと、保持は跳躍を*増幅する*機能になる
- **寿命は落とす。** 寿命はドリフトを見込んだ手当てだが、**測ったのはドリフトではなく跳躍**
  （roll/pitch は無傷、yaw も水中 61 分で 0 件）。**黙って保持をやめるのは現場での驚き**であり、
  しかもノブがもう 1 つ増える。**根拠が出てから足す**
- **`yaw_rate` でラッチ値を slew する**のは 1 行だが効く: **小さな修正のたびに bool を
  トグルしなくてよい**ので、保持とレートが 2 つのモードではなく連続になる

**置き場所は control。** 50 Hz の IMU yaw が要るうえ、**上位が落ちても保持し続けてほしい**から。
ラッチのリセットは `fix/attitude-reset-on-disarm` が前提（disarm で logic を初期状態に戻す）。

**残る小さな決めごと**: クランプ幅。**90° を初期値に**する（跳躍の実測は 169°、通常の
追従誤差は実測 6.7〜29.2° なので、その間なら誤爆しない）。**実機で振ってから確定すること。**

> **これで D-7 も決まる**: autonomy の `hold_yaw` は**この bool に置き換わる**（捨てずに移る）。
> `type_mask` は使われていないので捨てる。

### D-2. `Target.velocity` の単位 🔴

UI は**正規化スティック値**しか出さない（物理単位ではない）。autonomy は `cmd_target_vel_scale`
で m/s に直している。**control が `velocity` を m/s と解釈するのか正規化値と解釈するのか。**

割れると B-17 の再来（設定は入るのに意図と違う速度が出る）。**どちらでもよいが、決めて書くこと。**

> **調査（2026-09-29）— `dev-0921` の実装は「正規化値 `[-1, 1]`」として扱っている。**
> m/s でも N でもない。根拠（すべて `dev-0921`）:
> - **ゲインもクランプも一切無い**。`gate_controller.cpp:220-224` で購読値をそのままコピーし、
>   `attitude_controller.cpp:163-171` → `data_conversion.hpp:19-21` も恒等
> - `feed_forward.hpp:29-42` の `u` で、**並進 3 成分はゲイン無しで**、クォータニオンのベクトル部
>   （無次元・|·|≤1、`K_ATTITUDE=1.0`）と**同じ桁として並べられている**
> - `mixer.hpp:102-118` の配分行列 `a` は **`{0, ±1, ±√2}` の幾何だけ**。質量もスラスタの
>   定格推力もモータ定数も入っていない。出口で **`/√2`** している理由は、
>   **1 軸に 1.0 を入れたときスラスタ指令がちょうど ±1.0 に収まるようにするため**
>   （`velocity.x = 1.0` → 4 基とも ±1.0 = 飽和。テスト `test/cpp/.../mixer.cpp:96-103` と整合）
>
> ⚠ **ただし下流のラベルは「N」になっている（潜在的な単位の食い違い）。**
> `thruster_controller.cpp:77-83` の `duty_per_thrust` は
> `"Duty cycle per unit thrust [/N]"`、`docs/thruster_controller.md:35` も「推力[N]から
> Duty へ換算する係数」。だが**上流で N を作っている箇所はどこにも無い**。
> 既定 `duty_per_thrust: 1.0`（`params/controllers.yaml`）が実質**恒等のスルー**として
> 働いていて、N→duty の較正は**されていない**。`is_forward` / `thrust_sign` で踏んだ
> 「名前と実体がずれる」形と同じ。
>
> **→ 自然なのは「正規化値」。** 実装がすでにそうなっており、UI も正規化値しか出さないので、
> **m/s にすると control 側に新しいスケール係数を足すことになる**（未検証の経路が増える）。
> 決めたら **`Target.msg` にコメントで単位を書き、`duty_per_thrust` の `[/N]` ラベルも直す**。
> autonomy 側は `cmd_target_vel_scale` を**畳む**（control に合わせる = 移行方針どおり）。

> ⚠ 関連して **B-20**（`dev-0921` は同梱パラメータのままだと推力が一切出ない）を
> `known_issues.md` に起こした。**移植の前に踏むこと。**

### D-3. 浮力トリムを誰が持つか ✅ **決定（ユーザー 2026-09-29）**

**ソフトの浮力トリムは無くす。機体側（バラスト）で中性浮力に寄せる。**
**深度センサが間に合えば深度を閉ループにする**（それが本命の恒久対応）。

- control（`dev-0921`）に**浮力トリムは入れない**。autonomy の `net_buoy_up` 相当も**移植しない**
- したがって移植時の作業は「足す」ではなく「落とす」— `net_buoy_up` は畳む側に入る
- **深度センサは現状 載っていない**（ユーザー 2026-09-29）。**大会までに載る可能性はある**。
  したがって `/state/pressure` が来ていないのはソフトの不具合ではない — **復旧作業は不要**
- 閉ループが間に合わなければ、残る浮力は**機体側の調整のみ**で吸収する。
  ソフトの定数トリムで埋め戻さない（実機で未検証の値を既定にしない方針と整合）

> 参考: 現状の正浮力 +1.17 N は cap 0.25 の全推力の 15.6%。**バラスト調整前に潜航させると浮きます。**

> **裏付け（2026-09-29、9/13 bag）— 「ずっと余計な推力が出ている」の正体がこれだった。**
> 指令 velocity が **ちょうど 0** の区間で、**1 基あたり duty 中央 0.170（cap 0.25 の 68%）/
> 0.208（cap 0.40 の 52%）** が出ており、**和の鉛直成分が水平成分の 3〜4 倍・符号は一定**。
> 差動の姿勢フィードバックでは説明できない = 定常トリム。詳細と再現は
> `known_issues.md` **B-21**（`tools/idle_thrust_check.py`）。
> **→ トリムを外す利得は「コードが減る」ではなく「cap の半分以上が機動に戻る」。**
> さらに `classical.py:242` の記述どおりなら、要求ベクトルが純鉛直から外れることで
> **サーボの 180° 反転も減る**はず。

> **追補（ユーザー 2026-09-30）— トリムは「残さない」。狙いは残差ゼロ。**
> 「duty でそこまで正確に取れないんだから大体中性で諦める」+「残っていても余計な推力が
> 吸われるだけで良いことがない」。したがって:
> - **意図的な正浮力を残さない**（電源断で浮いてくる fail-safe 目的の残し方も**採らない**）。
>   理由: 残差の符号は水温・塩分・積載・抱き込んだ空気で日によって変わるので、
>   「常にわずかに正」という設計は**そもそも保てない**。目標は 0、符号は運用で見る
> - **精密な追い込みはしない。** 合否は「Σ\|duty\| ≈ 0」ではなく**桁が落ちたか**で見る
>   （現状 cap の 52〜68% が「大体中性」ですらない、という桁の話）
> - **浮心の前後（前浮心 / 後ろ浮心）も同じくバラストの前後移動で 0 に寄せる。**
>   これは純浮力と違い**定常の pitch トルク**として効き、質量だけでは直らない
> - **深度センサが載れば閉ループ、無ければオープンループでざっくり潜ってカメラで補正**。
>   下向きバイアスは **core が `Target.velocity.z` として出す**（control に定数を入れない）。
>   センサが載っても口は同じままで、中身が閉ループの出力に替わるだけ

> **sim 側の扱い（ユーザー 2026-09-30）**: 浮力・質量の値は元からざっくりなので、
> sim の決め打ち値（`buoyancy_offset_above_com: 0.010` など）を根拠に議論しない。
> ただし **DR ノブを足す作業は今はやらない** — **RL は封印**したので投資に見合わない。
> 実装と競技テストを先に回す。判明している穴だけ記録しておく:
> - `configs/umiusi.yaml:14` `displaced_volume: 0.012600` は中性 0.012480 より**意図的に正浮力**
>   （+0.12 kg ≈ +1.2 N）。上の「意図的な残しはしない」方針とは食い違っている
> - 純浮力の DR は既にある（`buoyancy_frac: 0.05` = ±0.63 kg ≈ ±6 N）ので**符号は両側に振れる**。
>   ただし幅が広すぎて**中性近傍が薄い**
> - **浮心の水平オフセット（前後・左右）は DR に存在しない。** `simulator.py:130`
>   `set_buoyancy_offset` は鉛直だけで、浮心は毎回 CoM の真上に自動配置される
>   （`simulator.py:117`）ため、**定常の pitch/roll トリムモーメントが構造的に常にゼロ**
> - ずらして回すハーネス自体は既にある（`tools/dr_ablation.py` は**古典制御**を 1 ノブずつ回す）

**残タスク**:
- **前提（センサ無し）で当日まで回せる形にする** — バラスト調整量を実機で決める。これが本線
- 深度閉ループは**載ってから**。載る前に実装を先行させない（未検証の経路を当日に増やさない）。
  ただし**置き場所だけ先に決めておく**と載ったときに安い → **core 側で `velocity.z` を作る**のが
  移行方針（姿勢＝control / 上位＝core）と整合。control の姿勢制御には触らない

### D-4. 並進を閉ループにするか

autonomy は速度観測器で閉ループ、control は開ループ。**観測器は深度センサが無いため指令からの
推定**（`/state/pressure` が実機に来ていない）。**開ループのままなら「指令した速度で進む」保証は無い**
が、そもそも観測器も推定値なので**精度の差は小さい**可能性がある。

**決めること**: 開ループで進めるか / 観測器も移植するか（移植は重い）

> **D-3 と同じセンサが律速**: 深度（z）を閉ループにできるかは `/state/pressure` 次第。
> 水平（x/y）はそのセンサでは埋まらないので、D-4 は独立に決める。

### D-5. 欠損スラスタ対応をいつやるか

lf が直ったので当面不要。ただし **`dev-0921` の mixer は固定行列なので、次に 1 基死んだら
同じ穴**（yaw 実現誤差 240%）。既存ブランチは**無い**（PR #317 は CLOSED・ブランチ削除済みだが、
**差分は `gh pr diff 317` で回収できる**。ただし `ff` 経路のもので mixer とは別物）。

**決めること**: 競技前にやるか / 後回しか

### D-6. `servo_sign` の正をどちらに置くか

`fix/deploy-hardening-2026-09` を入れると **control にも `servo_sign` ができる**。
autonomy にもある。**両方に −1 を入れると打ち消し合う。**

**決めること**: control 側を正にして autonomy 側を消す（移行方針と整合）でよいか

### D-7. `hold_yaw` / `type_mask` 相当の機能をどうするか ✅ **D-1 で決着**

`hold_yaw` は **`AttitudeTarget.hold_yaw`（bool）として control へ移る**。
`type_mask` は使われていないので**捨てる**。

### D-8. 推力ベクトルの反転をどう減らすか — **下流で捌くのではなく、上流で作らない**

いまの反転対策は**すべて配分の下流**にある（`mixer.hpp` の `SERVO_DIRECTION_DEADBAND` 2 deg /
`SERVO_REVERSAL_DEADBAND` 10 deg / `canonical_servo_angle` の折り返し）。
これは**出てしまった暴れを捌いている**だけで、暴れを作らない手当てではない。

**コストは摩耗より「止まっている時間」のほうが大きい。** 実測:

- **反転からの復帰は中央 0.4 s・p90 3 s**（起動直後の死に時間 2.7 s とは別）
- 実機の反転頻度は **0.02〜0.60 回/秒**（`lb` 0.02〜0.14 / `rb` 0.06〜0.60 / `rf` 0.02〜0.16）

→ 悪い方の 0.60 回/秒 × 0.4 s だと、**そのスラスタは時間の 2 割強を復帰に使っている**。
ESC の負荷という以前に、**推力が出ていない**。

**暴れの出どころは 3 つある。安い順に潰す。**

1. **動作点そのもの（= 浮力トリム）。D-3 でタダで直る見込み。**
   `classical.py:242` に「station holding では必要な力がほぼ**純鉛直**（＝浮力トリム）になるので
   `|phi|` の中央が 84 deg、59 % のステップで 80 deg 超、水平成分は 1.3 % のステップで符号が
   変わる — **そのたびに 180 deg のサーボ指令**」とある。**方位角特異点に張り付かせていたのが
   トリム。** バラストで釣り合わせれば要求ベクトルが特異点から離れる。
   **まず D-3 を入れてから測り直す。** 残った量を見ないうちに機構を足さない
2. **指令ベクトル自体のジッタ。** `kd` が 50 Hz の角速度に掛かるので、要求ベクトルの向きは
   毎周期揺れる。**向きが鉛直に近いほど、わずかな水平成分の揺れが符号反転になる**（1 の裏返し）。
   手当ては**水平成分のデッドバンド**（既にある 2 deg を広げる）か、**要求ベクトルの向きに
   一次遅れを入れる**。どちらも安いが、**遅らせた分だけ姿勢の応答も鈍る**
3. **配分が毎周期を独立に解いていること。** autonomy の `GeneralAllocator` は零空間のコストで
   特異点を避けていた（`prefer_deg`）。**control の `mixer.hpp` は固定行列なので零空間が無く、
   この手が使えない。** 入れるなら擬似逆行列の配分器ごと持ってくることになり、**重い**

**⚠ `tune/thruster-slew-4-0` はこの方針と逆を向いている。**
`max_duty_step_per_sec` を 1.0 → 4.0 に上げるブランチだが、これは**反転を速く・急にする**。
sim と揃えるという理由は分かるが、**sim 側の反転頻度が実機と 3〜90 倍ずれている**
（sim 1.85 回/秒 vs 実機 0.02〜0.60 回/秒）以上、**sim に合わせる根拠のほうが先に怪しい**。
**D-3 を入れて双方で測り直してから決める。**

**決めること**: 1 を入れて測り直したあと、2 を足すか（足すなら水平デッドバンドか LPF か）、
3 まで行くか。**2 と 3 は「残った量」を見るまで決めない。**

---

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
| `prefer_deg` / 零空間の特異点回避 | **D-8**（まず D-3 を入れて測り直してから） |

---

## 7. control 側の作業リスト（2026-09-29 時点）

上から順。**C-0 を飛ばすと以降が全部「動かない」に見える。**

### C-0. 【最優先・ブロッカー】B-20 — 同梱パラメータのままだと推力が出ない — **実装済み（2026-09-30、control `fix/servo-estimator-zero-blocks-thrust` @ `8a45706`）**

- [x] `params/controllers.yaml` の 4 箇所、`servo_max_angular_velocity` を **0 以外**にする → **4.0 rad/s**
- [x] 値はサーボの実力の**下限側**を入れる（小さすぎ = 収束が遅れるだけ / 大きすぎ =
      まだ動いている途中なのに到達したことにする）
      → **Hitec HS-646WP** のデータシート無負荷 300〜350 deg/s、`umiusi_sim` の水中ディレート
      250 deg/s の**下**に置いた 229 deg/s。**実測ではない**
- [x] **0 のまま起動したら警告を出す**（今は黙って推力 0 になる。現場で気付けない）
      → `thruster_controller.cpp` の `on_configure` で `RCLCPP_ERROR`。
      **configure は失敗させない**（既存挙動を変えないため。鳴らすだけ）
- [x] 0 を二度と積めないよう `test/python/test_controllers_params.py` で固定（B-17 の方針）。
      わざと 0 に戻して落ちることを確認済み。**build OK / 195 tests pass**
- [x] `preflight.py` の検品に足した → ただし **`servo/estimated_angle` の NaN ではなく
      `servo_max_angular_velocity` の param を 4 基から読む**方式にした。
      理由: NaN 検査は state インタフェースの publish 経路に依存するが、param なら
      **現場で `param set` により 0 に戻された場合も捕まえられる**（リポジトリのテストでは捕まらない）。
      未宣言（= B-20 修正前の古いバイナリ）は WARN で区別する

**新たに分かったこと**: `thruster_controller.cpp:354-363` は `servo_allowed` が false の間
estimator を **reset** するので、候補区間は **disarm ごとに開き直る**。arm から最初の推力までに
最悪 **π/4.0 = 0.79 s** が毎回乗る。**B-14 の起動死に時間 約 2.7 s の候補の 1 つ**だが、
あちらは autonomy 経路での実測なので同一視しない（移植後に再測）。

詳細と根拠は `known_issues.md` **B-20**。

### C-1. 【D-1】yaw 保持を足す — **実装済み（2026-09-30、ブランチ `feat/hold-yaw`）**

- [x] `sinsei_umiusi_msgs/AttitudeTarget` に **`bool hold_yaw`** を足す（既定 false = 今の挙動）
- [x] `logic::attitude::AttitudeFeedback` にラッチを持たせる:
      false→true のエッジで実測 yaw をラッチ / true の間は `latched += yaw_rate * dt` /
      `|誤差| > yaw_hold_relatch_error`（既定 **90°**）で現在値へラッチし直し
- [x] ラッチを `reset()` で捨て、`FeedBack::init()` から呼ぶ
- [x] テスト 10 件（**跳躍のテストは実機 bag の −169.03° を使う** / 通常の追従誤差 29.2° で
      誤爆しないこと / ±π の折り返し / hold を落とすとラッチを捨てること）。**201 tests pass**
- [x] `docs/attitude_controller.md` に `FeedBack` の yaw の節を追加
- [ ] **`fix/attitude-reset-on-disarm` を先に入れる** — `reset()` は書いたが、disarm で
      `init()` が走る経路自体が `dev-0921` にはまだ無い
- [ ] **起動の死に時間 2.7 s の直後にラッチしない**（arm 直後の姿勢は当てにならない）。**未着手**
- [ ] ラッチし直したときの **WARN ログ**。`yaw_was_relatched()` は用意したが、
      `AttitudeController` 側でまだ拾っていない。**未着手**
- [ ] `kp_yaw_hold` / `yaw_hold_relatch_error` を実機で振って確定する（**実機未検証**）

> `dev-0921` は姿勢ゲインをそもそもパラメータ化していない（`AttitudeFeedbackGains` の既定値が
> そのまま効く）ので、この 2 つも同じ形にしてある。**`param set` で変えられると書かない**（B-17）。

### C-2. 【D-2】単位を書いて、嘘のラベルを直す

実装は変えない（**すでに正規化値として動いている**）。**書くだけ。**

- [ ] `sinsei_umiusi_msgs/msg/Target.msg` の `velocity` に
      **「正規化指令 [-1, 1]。物理単位ではない」**とコメントを入れる
- [ ] `thruster_controller.cpp` の `duty_per_thrust` の記述 `"Duty cycle per unit thrust [/N]"`
      と `docs/thruster_controller.md:35` を直す。**N→duty の較正はされていない**
- [ ] `params/controllers.yaml` の `# Duty cycle per unit thrust [/N]` も 4 箇所直す
- [ ] autonomy 側の `cmd_target_vel_scale` は**畳む**（control に合わせる）

### C-3. 【D-3】浮力トリムは足さない

- [ ] **作業は「足さないこと」を確認するだけ。** `dev-0921` にトリムが無いことは確認済み
- [ ] 代わりに**バラスト調整**（機体側）。調整後の検証は `tools/idle_thrust_check.py` で
      「指令 0 で `Σ|duty| ≈ 0`」（B-21）

### C-4. ブランチの整理 — **一部実施済み（2026-09-30）**

> ⚠ **以前ここに書いた本数は誤り**だった。ローカルの `main` が `origin/main` より **30 コミット
> 遅れて**おり、その古い `main` を基準に数えていた。正しい基準は **`origin/main` = `c8cd1fa`**。

**実施した整理（ローカルのみ。リモートには触っていない）**:

- ローカル `main` を `origin/main` へ早送り（30 遅れ → 0）
- **削除 3 本**（いずれも内容が別の場所に完全に残っている）:
  - `fix/ff-thrust-sign` — PR #307 で `main` にマージ済み
  - `feat/rl-attitude-logic` / `feat/rl-economy-gate` — **全パッチが `feat/rl-attitude` に
    同内容で入っている**（`git cherry` で確認。origin にも残っている）
- **古い worktree を 6 つ削除**（`/tmp/.../scratchpad/wt-*`、すべて未コミット変更なし）。
  これがブランチ削除を塞いでいた

**残り（`origin/main` 比のコミット数）**:

| 本数 | ブランチ | 扱い |
|---:|---|---|
| 10 | `dev-0921` | **本体。PR 化する** |
| 5 | `fix/deploy-hardening-2026-09` | PR #308 と重複。統合してから |
| 21 | `feat/imu-sanity` | A-1 対策の移植。**C-1 のクランプと役割が重なる。順序注意** |
| 20 | `test/hw-verify` | **ローカルのみ（origin に無い）。消さないこと。** 内容は他ブランチの リベース版だが、3 件中 2 件は patch-id が一致せず**差分がある** |
| 18 | `fix/attitude-reset-on-disarm` | **C-1 の前提。先に入れる** |
| 18 | `fix/torch-link-leaks-into-camera-node` | 中身を確認する |
| 2 | `tune/thruster-slew-4-0` | **D-8 と逆を向いている。保留** |
| 2 | `fix/can-servo-update-rate` | サーボ CAN の更新レート・書き込み失敗の扱い |
| 2 | `fix/actuator-limits-on-direct-cmd` | 指令の出所によらず歯止めを効かせる（B-12 と同系） |
| 1 | `fix/hardware-health-flags` | headlights / indicator_led の health |
| 19 | `feat/rl-attitude` | **封印。触らない** |
| — | **`feat/hold-yaw`（新規）** | **C-1 の実装。`dev-0921` から分岐** |

> `fix/deploy-hardening-2026-09` / `tune/thruster-slew-4-0` / `fix/actuator-limits-on-direct-cmd`
> は**同じコミット（`8ae6ee7`）を共有している**。個別に入れると重複する。

**リモートの整理（2026-09-30 実施）** — ユーザー方針「**自分が作ったもののみ**」に従った。

- **削除 2 本**（どちらも satoimo 作・open PR 無し・全パッチが `origin/feat/rl-attitude` に同内容で残る）:
  - `origin/feat/rl-attitude-logic`（退避 SHA `ee044d5`）
  - `origin/feat/rl-economy-gate`（退避 SHA `047fe08`）
- **push 2 本**: `feat/hold-yaw`（control / msgs の両方）

**触っていないもの**:

| 種別 | ブランチ |
|---|---|
| **共同開発者 (yk4to) 作** | `alexandrite` #314 / `harmony` #332 / `servo-angle-estimation` #331 / `codex/actuator-can-support` #316 / `dev-0921`（**全部 open PR あり**） |
| satoimo 作だが生きている | `feat/imu-sanity` / `feat/rl-attitude`（封印だが**唯一の保存先**）/ `fix/attitude-reset-on-disarm` / `fix/can-servo-update-rate` / `fix/deploy-hardening-2026-09` / `fix/hardware-health-flags` / `tune/thruster-slew-4-0` |
| satoimo 作・open PR あり | `fix/actuator-limits-on-direct-cmd`（**PR #308**） |

> ⚠ **`fix/torch-link-leaks-into-camera-node` は消してはいけない。** 全パッチが
> `feat/rl-attitude` にも入っているので一見重複だが、**そちらは封印**なので、
> 「libtorch がカメラノードへ漏れる」実バグの修正が**マージできる形で残っている唯一の場所**。

**msgs リポジトリ**: satoimo 作は `fix/servo-angle-unit-comment`（1 コミット・未マージ・PR 無し）
だけで、これは `ThrusterOutput.angle` の単位コメントを rad → DEGREES に直す**実修正**なので残した。
残りは全部 yk4to 作（`harmony` #11 / `remove-battery-current` #10 / `servo-angle-estimation` / `dev-0921`）。

### C-5. 移行で autonomy 側から消えるもの（利得の確認用）

- **`/cmd/direct` の publisher**（B-12）← **移植の最大の利得**
- `net_buoy_up` の定常トリム（B-21・D-3）
- `cmd_target_vel_scale`（D-2）
- `cmd_target_yaw_mode` / `_yaw_rate_scale` / `_yaw_lead_max`（D-1）
- `servo_sign` の二重管理（D-6）

### C-6. 残っている判断

- **D-4**（並進を閉ループにするか）/ **D-5**（欠損スラスタ）/ **D-6**（`servo_sign` の正）
- **D-8**（推力ベクトルの反転をどう減らすか）— **D-3 を入れて測り直すまで決めない**
- B-19（`imu_timeout` / `vel_timeout` / 例外ガード）を control へ移す — 判断不要、作業のみ

---

## 8. 後進（バック）はどう出るか

D-2 で「velocity は正規化値」と決めたので、**後進は `velocity.x` を負にするだけ**。
経路は前進と完全に対称で、専用の仕組みは要らない。

1. `Target.velocity.x < 0` → `u[3] < 0`（`feed_forward.hpp`、ゲイン無し）
2. 配分行列の 4 列目が `{-√2, 0, -√2, 0, +√2, 0, +√2, 0}` なので、水平成分が前進と逆符号
3. `esc_thrusts[i] = projected / √2` → **duty が負**（`ThrusterOutput.duty_cycle` は -1.0〜1.0）
4. `LinearAcceleration` が `is_forward` の符号を掛け、`max_duty` でクランプ、
   `max_duty_step_per_sec` でスルーレート制限

**注意点**:

- **サーボ（±90°）は後進に使わない。** 向きは duty の符号で作る。サーボを 180° 回して
  前進推力で下がろうとすると、**±90° の折り返し（方位角特異点）に突っ込む** — B-14 / B-21 の形
- **モデルは前後対称だが、実機のペラは対称ではない。** `classical.py` の推力則は
  `F = sign(u) * |u|^exp * thrust_per_cmd` で**符号に対して対称**。実機の後進推力が
  前進より弱いなら**その差はどこにも入っていない**。`thrust_calibration.md` のとおり
  `thrust_curve_exp` 自体が未較正（sim のプラントは 2.0、`feedforward_allocation` の既定は 1.0）
- **後進で「前に進む」なら符号の問題**。`field_card.md` / `scenario_run.md` のとおり、
  **ハードを直さずに止めること**（`is_forward` とバンドルの `thrust_axes` の片方だけ直すと
  6 自由度すべて反転する）
- 後進中の yaw 保持は**そのまま効く**（C-1 のラッチは velocity と独立）
