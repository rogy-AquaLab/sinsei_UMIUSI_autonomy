# 推力の同定 — ログから後で分析する

専用の較正実験を組まず、**通常の走行のログから後で推定する**。そのためにログ側を整える。

## なぜ要るか

経路ごとに推力の仮定が違う。**同じ指令でも出る duty が違う。**

| 経路 | 換算 | 出どころ |
|---|---|---|
| direct + RL | `F = abs(u)^2 * 30` (2 乗) | バンドルの `action_contract` |
| direct + navigator | **線形** | `feedforward_allocation` の `thrust_curve_exp` 既定 1.0。**2026-09-13 にコードで裏取り済み** |
| target (core) | **線形** | control の `duty_per_thrust: 1.0` |

`max_duty` 0.25 の範囲では 2 乗と線形で **4 倍**の差になる。**経路をまたいだ実験結果は
そのままでは比較できない。**

さらに sim の `configs/umiusi.yaml` 自身が `thrust_per_cmd` と `thrust_curve_exp` について
**`BOTH VALUES ARE UNCALIBRATED and confounded with each other`** と書いている。

**2026-09-13 にコードで確認した** (2026-09-09 時点の「パッケージが見つからない」という
記述は誤りだった。`umiusi_perception` は `umiusi_sim/packages/perception/` にあり、機体には
そこから wheel で入る):

- `feedforward_allocation` は `umiusi_perception/control.py`。引数 `thrust_curve_exp` の
  **既定は 1.0** で、そのとき ESC 指令を逆カーブで事前に歪めない (`u = sign(t)|t|^(1/exp)`)。
- **`navigator_node` はこの引数を渡していない**ので、direct + navigator の経路は**線形**。
  上の表の「線形」はこれで裏が取れた。
- 一方プラントの実体は `thrust_curve_exp: 2.0` (未較正) なので、**この経路だけ推力の仮定が
  プラントと食い違っている**。`feedforward_allocation(..., thrust_curve_exp=2.0)` を渡せば
  事前歪みが入って揃うが、**exp 自体が未較正なので先に同定するのが順序**。
- `thrust_curve_exp` / `thrust_per_cmd` を読んでいるのは `umiusi_rl_control/mode_action.py` と
  `umiusi_perception.classical` (バンドルの契約経由)。

> **シナリオを `command_mode:=setpoint` で回すとこの食い違いは消える。** 配分が
> `umiusi_perception.classical` の `GeneralAllocator` (契約の exp を使う) に一本化されるため。
> `direct` を使い続ける場合だけ上の話が効く。

## 運用時の max_duty — 既定 0.25 だが実運用は 0.3〜0.4 の可能性がある

到達速度の議論をするときは **cap がいくつだったか**を run ごとに確定させること。
sim の解では `thrust_curve_exp` を 2.0 に固定したままでも、cap 0.25 で 0.40 kt、
cap 0.40 で 0.70 kt と **1.75 倍**変わる。「exp が汚染されている」と「運用 cap が高い」は
排他ではなく、両方効いている可能性がある。

| 経路 | 既定 | 備考 |
|---|---:|---|
| `rl_attitude_node` / `rl_attitude.launch.py` | 0.25 | |
| `navigator_node` | 0.25 | **この経路の歯止めはこれだけ** |
| `umiusi_autonomy/config/autonomy.yaml` | 0.25 | |
| control `rl.max_duty` | 0.25 | |
| control `thruster_controller.max_duty` | 0.5 | **direct 経路では効かない** (B-12) |

**既定が 0.25 でも、ドキュメントは上げることを推奨している**:

- `rl_attitude.launch.py` の使用例に `max_duty:=0.4` が直書きされている
- 同ファイルに「max_duty 0.3 以上を推奨 — 0.2 で降下できないのは …」
- `depth_supervisor` は **max_duty 0.4 が前提**で、0.3 未満だと警告を出す

したがって **過去の run が 0.25 だったとは限らない**。bag の `duty_cycle` (指令のエコー) を
見れば run ごとに確定するので、到達速度を語る前にそれを確認すること。

### cap をいくつにすると比例制御のままでいられるか (2026-09-12 実測)

当日そのまま走る `classical_attitude_node` + 同梱 `classical_bundle.json` に偽 IMU で
roll を与え、**指令 duty が cap に張り付き始める姿勢誤差**を測った (機体不要。PC 上で
ノードを起動し `/state/imu` をこちらから流しただけ)。

| cap | duty が cap に張り付き始める roll | 20° 傾けたときの飽和 |
|---:|---:|---|
| 0.25 | **10°** | 1〜2 基が張り付き |
| 0.30 | 8°（散発）/ 15° 以降は恒常 | 1 基が張り付き |
| 0.40 | **25°** | 飽和なし |

**cap 0.25 の比較は姿勢誤差 10° を超えた時点で比例制御ではない** — 古典も RL も
「上限を出しっぱなし」になるので、そこから先はゲインや方策の差が指令に出ない。
**比較として意味があるのは cap 0.4。** 20° まで飽和しない。

一方 **サーボ角はどの cap でも ±88〜90° に張り付く** (roll 0° でも)。これは浮力に抗して
推力を下に向けるためで、**cap を上げても解消しない** — 向きの問題であって大きさの問題では
ないため。`known_issues` の「サーボが振り切る」はこれで、故障ではない。

> 測っているのは**開いた系での指令の飽和**であって、閉ループの挙動ではない
> (静的な姿勢を与えて指令を読んだだけで、プラントは回していない)。
> 「どの姿勢誤差から比例制御でなくなるか」の判定にだけ使うこと。
>
> **2026-09-12 のプール run が飽和していた原因は cap ではない。** あの run は推力の符号が
> 反転していて yaw 誤差が 180° に張り付いており (known_issues B-14)、その巨大な誤差が
> duty を上限へ押し付けていた。**cap を上げても直らない。符号が先。**
> 逆に、duty -> rpm の回帰は大きさの話なので**符号が反転していても使える**
> (`tools/duty_rpm_fit.py`)。符号確認の run でスラスタを回すので、そのついでに取れる。

## いま bag に何が入っているか

`/state/thruster_state_all` の中身:

| 項目 | 実測か |
|---|---|
| `duty_cycle` | **指令のエコー**。実測ではない (msg にそう書いてある) |
| `angle` | **指令のエコー**。サーボに位置フィードバックが無い |
| `rpm` | **実測** |

**`rpm` が実測で入っているのが効く。** `duty -> rpm -> 推力` の 2 段のうち前段は既に見える。

**推力 ∝ rpm^2 は物理として堅い**ので、`duty -> rpm` の回帰だけで曲線の形が判定できる:
線形なら `thrust_curve_exp ≈ 2.0`、飽和していれば 2.0 未満。**秤も新規実験も要らない。**
併せて原点付近のデッドゾーンの有無も見ること (sim は未モデル化)。立ち上がり数秒は固着の
影響が出るので除外する。

### 現時点で回帰可能な bag は存在しない (2026-09-11)

**この回帰は今はできない。** 同じ探索を繰り返さないために状況を残す:

- 手元 (`mujoco_ws/data/`) の `20260821-080906-imu-motion` は **disarm 状態**
  (`esc_mode 0` / duty 全ゼロ / rpm 全ゼロ)。IMU を手で振る試験なのでスラスタを回していない。
- **これ以上の bag は存在せず、機体も接続されていないので取り出せない** (2026-09-11 のユーザー回答)。

判定用のスクリプトは `tools/duty_rpm_fit.py` に置いてある。走行 bag が取れた日に
そのまま流せる。上の bag に対しては「Runnable のサンプルが無い」と報告して終わる。

```bash
python3 tools/duty_rpm_fit.py <bag-dir>
```

**bag 経由が消えたので、`thrust_curve_exp` を確定する道は 2 つ**。どちらも実機作業が要る:

1. **重力アンカー方式** — 空中重量と水中の見かけ重量 (ばね秤) の差が `ρVg` として絶対値で
   出る。定常ホバリングの duty を読めば `thrust_per_cmd = B / (4 * u_hover^exp)`。
   指数はバラスト掃引で既知の力を振って取る。秤 1 個とバラストだけで治具が要らない。
2. **電流プロキシ** — 下の「VESC の実 duty と電流」。BLDC のトルクは電流にほぼ比例するので
   **通常の走行だけで曲線の形が取れる**。絶対値のアンカーは 1 の秤 1 点で足りる。

**2 を先に入れると 1 の測定回数が減る。**

## 足りないもの: VESC の実 duty と電流

VESC は `CAN_PACKET_STATUS` で **実 duty と電流を返している**。control の `vesc_model.cpp` は
それをデコードしているが、`state::thruster::esc` に出していないので (`Rpm` / `Voltage` /
`WaterLeaked` だけ) **bag に残らない**。

**2026-09-06 に実機で確認済み**: `CAN_PACKET_STATUS` (id `0x0009xx`) は **4 基とも 50 Hz で
バスに流れている**。`vesc_model.hpp` の `PacketStatus` は `erpm` / `current` / `duty` の
3 つをデコードしていて、`can_model.cpp` が `erpm` だけを `state::thruster::esc::Rpm` に
転送している。**取りこぼしているのは転送段だけで、データは既に届いている。**

出すと:

- **電流が推力の車載プロキシ**になる。BLDC のトルクは電流にほぼ比例するので、
  秤なしで曲線の形が取れる。絶対値のアンカーが要るなら秤で 1 点だけ
- **指令 duty と実 duty の差**が見える。VESC 内部のランプが分かるので、sim の
  `thrust_slew_range` の下限を推測ではなく実測で置ける

control 側の小改修。**入れれば通常の走行が全部較正データになる。**

## 分析するときに見ること

### 低 duty の非線形

**RL は `max_duty` 0.25 の範囲でしか動かない。** デッドゾーンがあるなら、方策が使う領域の
大部分が非線形ということになる。**sim はデッドゾーンをモデル化していない** (`umiusi.yaml` に
該当するキーが無く、`F = abs(u)^exp * k` の純粋なべき乗のみ)。

`rpm` を duty に対してプロットして、原点付近で折れていないか見る。

### 回り始めのばらつき

BLDC は錆や固着で **起動直後だけ回らない / 回り方が揃わない**ことがある。

**これは毎回同じ特性ではないので、係数として同定しない。** 曲線に折り込もうとすると、
たまたまその日の固着を焼き込むことになる。分析では次のように扱う:

- 走行の**最初の数秒を当てはめから外す**
- 4 基の間で `rpm` の立ち上がりが揃っているか見る。1 基だけ遅れていたら固着を疑う
- 揃わない日のデータは係数の推定に使わない

**運用側の対策**: 少し動かすとスムーズになるので、**走行前に低 duty で数秒回す**。
`teleop_keyboard` で `w` を軽く入れて戻すだけでよい。これはログではなく手順の話。

### 姿勢応答からの同定

秤で静推力を測るのはコストが重い。**IMU の応答から推定するほうが目的に近い** —
sim を実機に合わせたいなら、合わせるべきは静推力ではなく閉ループの応答だから。

素材になるもの:

- `/state/imu` 50 Hz (姿勢・角速度)
- `/cmd/direct/...` の指令 (50 Hz)
- `/state/thruster_state_all` の `rpm`

`tools/thruster_cmd.py excite` は有界ランダム励起を 120 s 出すもので、**同定用に作られている**
(docstring に world model 用とある)。専用実験を組むならこれだが、**通常の走行でも
指令が十分に動いていれば素材になる**。

## 反映先

| 決まった値 | 反映先 |
|---|---|
| `thrust_per_cmd` / `thrust_curve_exp` | sim の `configs/umiusi.yaml` |
| `duty_per_thrust` | control の `params/controllers.yaml` |
| navigator の `thrust_curve_exp` | `navigator_node` の `feedforward_allocation` 呼び出し (いま渡していない = 線形)。**`command_mode:=setpoint` なら不要** |

**control 側は線形なので 2 乗カーブとは原理的に一致しない。** 運用範囲で線形近似を取るか、
control 側も 2 乗にするかは実測を見てから決める (`LinearAcceleration` の名前ごと変わる話)。

## 関連

- `known_issues.md` A-17 — `max_duty` と転覆余裕。ただしそこに書いてある
  `v_max ≈ 0.68 * max_duty` は **cap 0.25 近傍でしか合わない線形近似**で、
  cap 0.4 で 24% / cap 0.5 で 41% 過小になる (2026-09-11、sim 側でプラント解と比較)。
  正しくは `lin*v + quad*v^2 = 4*k*cap^exp` の正根。cap を上げた運用で使わないこと
- `known_issues.md` B-12 — `/cmd/direct` は logic を通らない
- `logging.md` — bag に何を入れるかの方針
