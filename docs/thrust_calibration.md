# 推力の同定 — ログから後で分析する

専用の較正実験を組まず、**通常の走行のログから後で推定する**。そのためにログ側を整える。

## なぜ要るか

経路ごとに推力の仮定が違う。**同じ指令でも出る duty が違う。**

| 経路 | 換算 | 出どころ |
|---|---|---|
| direct + RL | `F = abs(u)^2 * 30` (2 乗) | バンドルの `action_contract` |
| direct + navigator | **線形** | `feedforward_allocation` の `thrust_curve_exp` 既定 1.0。`navigator_node.py:183` は渡していない |
| target (core) | **線形** | control の `duty_per_thrust: 1.0` |

`max_duty` 0.25 の範囲では 2 乗と線形で **4 倍**の差になる。**経路をまたいだ実験結果は
そのままでは比較できない。**

さらに sim の `configs/umiusi.yaml` 自身が `thrust_per_cmd` と `thrust_curve_exp` について
**`BOTH VALUES ARE UNCALIBRATED and confounded with each other`** と書いている。

**2026-09-09 に grep で確認した範囲** (`ros2_ws/src` 全体):

- `thrust_curve_exp` / `thrust_per_cmd` を読んでいるのは **`umiusi_rl_control/mode_action.py` だけ**
  (`rl_attitude_node.py` はログ表示のみ)。上の表の「direct + RL」以外は契約値を参照していない。
- `navigator_node` は `umiusi_perception.control.feedforward_allocation` を呼んでいる。
  **`umiusi_perception` はこのリポジトリに無い**ので、その既定が 1.0 であることは
  ここからは確認できていない。上の表のその行は未検証として扱うこと。

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

**使える bag はまだ回収できていない** (2026-09-09 時点)。手元 (`mujoco_ws/data/`) にある
`20260821-080906-imu-motion` は **disarm 状態で録られていて** `esc_mode 0` /
duty 全ゼロ / rpm 全ゼロなので回帰には使えない (IMU を手で振る試験)。
必要なのは Pi の `~/runs/` にある走行 run:

```
20260825-154411-exp1-spin
20260825-181310-0825_run
20260825-183035-0825_run2
20260821-174515-servo-debug (+ attitude-only 2 本)
```

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
| navigator の `thrust_curve_exp` | `navigator_node.py:183` の呼び出し (いま渡していない) |

**control 側は線形なので 2 乗カーブとは原理的に一致しない。** 運用範囲で線形近似を取るか、
control 側も 2 乗にするかは実測を見てから決める (`LinearAcceleration` の名前ごと変わる話)。

## 関連

- `known_issues.md` A-17 — `max_duty` と転覆余裕、到達可能速度 `v_max ≈ 0.68 * max_duty`
- `known_issues.md` B-12 — `/cmd/direct` は logic を通らない
- `logging.md` — bag に何を入れるかの方針
