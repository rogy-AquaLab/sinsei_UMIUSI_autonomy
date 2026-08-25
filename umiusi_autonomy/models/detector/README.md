# 同梱の検出器チェックポイント

風船検出器 (TinyBalloonNet) の学習済み重み。`perception_node` が読む。
RL 姿勢制御のポリシー (`umiusi_rl_control/models/`) とは**別物**なので注意。

学習は `Umiusi_sim` 側 (`tools/perception_train.py`)。ここに置いてあるのは、
**clone しただけで動かせるようにするための既定値**。

| ファイル | 学習データ | 推奨 conf | val の F1 | 用途 |
|---|---|---:|---:|---|
| **`camp_real2.pt`** (既定) | camp_real + 8/25 プール実写 265 枚 | **0.4** | **0.80** | **競技はこれ**。実プールの誤検出を潰した版 |
| `camp_real.pt` | 実写 161 枚 | 0.3 | 0.44 | 旧版。A/B 比較用に残してある |
| `camp_mix.pt` | sim 1000 + 実写 161 | 0.3 | — | sim 寄り。sim_eval の F1 は最良 (0.47) |

`cfg` は `width=16 / input_size=256`。`conf_thresh` は **`camp_real2` だけ 0.4**、
他は 0.3 (チェックポイントに格納されているので、`conf_thresh` パラメータを
指定しなければ自動でその値が使われる)。

## `camp_real2` で何が変わったか

8/25 の水中 run で、**camp_real は実プール映像で実用水準に無い**ことが分かった
(4.6 個/枚の誤検出、画面右下に動かない固定誤検出)。その run から切り出した 265 枚
(黄 26 箱 + 背景ネガ 239 枚) をハードネガティブとして継続学習したものが `camp_real2`。

val (旧 real_val 25 枚 + プール 46 枚) での比較:

| | precision | recall | F1 | プール上の FP | 右下の固定 FP |
|---|---:|---:|---:|---:|---:|
| `camp_real` @0.3 (8/25 に実機で使った設定) | 0.29 | 0.91 | 0.44 | 267 | 16 |
| **`camp_real2` @0.4** | **0.78** | 0.82 | **0.80** | **3** | **0** |

recall は 0.91 → 0.82 とわずかに落ちるが、**precision が 0.29 → 0.78**。
FSM は誤検出に引っ張られてロックし損ねるので、この交換は妥当。

## 使い分け

既定は `camp_real2.pt`。切り替えは launch 引数で:

```bash
ros2 launch umiusi_autonomy core_autonomy.launch.py \
    model_path:=$(ros2 pkg prefix umiusi_autonomy)/share/umiusi_autonomy/models/detector/camp_real.pt
```

## 注意

* **`input_size` を下げると速くなるが精度を大きく失う** (256→192 で F1 0.69→0.55、
  recall 0.72→0.54)。`docs/performance_tuning.md` を参照
* `conf_thresh` は速度に効かない (CNN の推論コストは画像の中身に依らない)。
  誤検出を減らす目的でのみ使う
* 学習データや評価の詳細は `Umiusi_sim` の `ai/balloon/campaign_results.md`
  (`camp_real2` は 2026-08-26 の節)
