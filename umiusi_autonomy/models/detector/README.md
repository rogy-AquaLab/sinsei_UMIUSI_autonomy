# 同梱の検出器チェックポイント

風船検出器 (TinyBalloonNet) の学習済み重み。`perception_node` が読む。
RL 姿勢制御のポリシー (`umiusi_rl_control/models/`) とは**別物**なので注意。

学習は `Umiusi_sim` 側 (`tools/perception_train.py`)。ここに置いてあるのは、
**clone しただけで動かせるようにするための既定値**。

| ファイル | 学習データ | real_val の F1 | sim_eval の F1 | 用途 |
|---|---|---:|---:|---|
| **`camp_real.pt`** | 実写 161 枚 | **0.80** | 0.33 | **実際の水中はこちらが強い**。競技はこれ |
| `camp_mix.pt` (既定) | sim 1000 + 実写 161 | 0.69 | **0.47** | 両対応。実機で通しの動作確認をしたのはこちら |

`cfg` は両方とも `width=16 / input_size=256 / conf_thresh=0.3`。

## 使い分け

既定は `camp_mix.pt`。切り替えは launch 引数で:

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
