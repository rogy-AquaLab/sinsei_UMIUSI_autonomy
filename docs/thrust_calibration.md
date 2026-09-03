# 推力曲線の較正

`thrust_per_cmd` と `thrust_curve_exp` を実測で決める手順。

## なぜ要るか

sim と control が推力について違うことを仮定している。

| | モデル | 値 |
|---|---|---|
| sim / RL | `F = sign(u) * abs(u)^thrust_curve_exp * thrust_per_cmd` | `2.0` / `30.0 N` |
| control (`LinearAcceleration`) | `duty = thrust * duty_per_thrust` (線形) | `1.0` |

sim の `configs/umiusi.yaml` 自身が **`BOTH VALUES ARE UNCALIBRATED and confounded with
each other`** と書いている。2 つの係数が交絡していて、いまの水中データだけでは分離できない。

RL を control の `attitude_controller` に載せると、RL が出す推力 [N] を control が duty に
戻すので、**2 つの曲線が一致していないと出る力が変わる**。曲線を 1 つに決めるのがこの実験。

## 用意するもの

- 推力を測る秤（機体を固定できること）
- `tools/thruster_cmd.py`
- **`--allow-full` を使うので、機体は必ず固定してから**

## 手順

### 1. ゼロ点

```bash
./tools/record_run.sh --name thrust-cal-01
```

秤をゼロにする。`record_run.sh` はスタックより先に起動してよい。

### 2. スイープ

```bash
python3 tools/thruster_cmd.py sweep --ch lf --allow-full
```

`duty ±0.2 .. ±1.0` を各 `dwell` 秒ずつ駆動し、間に停止を挟む。
**停止のたびに秤のゼロを確認する**（ドリフトすると全点がずれる）。

各 duty で秤の読みを記録する。4 基それぞれ（`--ch lf/lb/rb/rf`）。

### 3. 当てはめ

`F = a * abs(u)^b` を最小二乗で当てる。

- `b` が 2.0 に近ければ propeller law が妥当。`thrust_curve_exp` はそのまま
- `a` が `thrust_per_cmd`。sim の 30.0 とどれだけ違うかを見る
- **正負で非対称なら別々に当てる**。ペラは前後で効率が違う

### 4. 反映先

| 決まった値 | 反映先 |
|---|---|
| `thrust_per_cmd` / `thrust_curve_exp` | sim の `configs/umiusi.yaml` |
| `duty_per_thrust` | control の `params/controllers.yaml` |

**control 側は線形なので、2 乗カーブとは原理的に一致しない。** 運用範囲
(`max_duty` 0.25 まで) で最小二乗の線形近似を取るか、control 側も 2 乗にするかの判断が要る。
ここは実測を見てから決める。

## 自動化できるようにするには

いまは秤の読みを人が記録している。**VESC は CAN で実 duty と電流を返しており**、
`vesc_model.cpp` の `CAN_PACKET_STATUS` でデコードしているが `state::thruster::esc` に
出していない（`Rpm` / `Voltage` / `WaterLeaked` だけ）。

これを出すと:

- 電流が推力の車載プロキシになり、**秤なしで曲線の形が取れる**（絶対値は秤で 1 点あれば足りる）
- 指令 duty と実 duty の差が見えるので、**VESC 内部のランプが分かる**。
  sim の `thrust_slew_range` の下限を推測ではなく実測で置ける

control 側の小改修。次の実験の前に入れられると、この手順が一度で済む。

## 関連

- `known_issues.md` A-17 — `max_duty` と転覆余裕。到達可能速度 `v_max ≈ 0.68 * max_duty`
- `known_issues.md` B-12 — `/cmd/direct` はこの経路。`thruster_cmd.py` は素の指令を出す
- issue #18 の実験 4
