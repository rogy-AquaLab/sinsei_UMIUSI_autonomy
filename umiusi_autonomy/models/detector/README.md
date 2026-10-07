# 同梱の検出器チェックポイント

風船検出器 (TinyBalloonNet) の学習済み重み。`perception_node` が読む。
RL 姿勢制御のポリシー (`umiusi_rl_control/models/`) とは**別物**なので注意。

学習は `Umiusi_sim` 側 (`tools/perception_train.py`)。ここに置いてあるのは、
**clone しただけで動かせるようにするための既定値**。

| ファイル | 学習データ | 推奨 conf | val の F1 | 用途 |
|---|---|---:|---:|---|
| **`balloon_F320_20261007.pt`** (既定) | 上の 1348 枚 + 10/03 プールの見え方を掛けた sim 500 枚、入力 320 | **0.30**（赤だけ **0.50**） | JAMSTEC+他チーム 0.46、10/03 未見 0.53 | **2026-10-08 から既定**。下の節 |
| `balloon_F320_20261003.pt` | 1348 枚 (自チーム + 他チームの JAMSTEC 映像)、入力 320 | 0.3 (+ `min_confidence` 0.40) | JAMSTEC 0.48 | 10/03〜10/08 の既定 |
| `balloon_F256_20261003.pt` | 同上、入力 256 | 0.3 (+ 0.40) | JAMSTEC 0.40 | 予備。**認識が 4 Hz を大きく下回るとき** |
| `camp_real.pt` | 実写 161 枚 | 0.3 (+ 0.45) | 0.44 | 10/01〜10/03 の既定 |
| `camp_real2.pt` | camp_real + 8/25 プール実写 265 枚 | 0.4 | 0.80 | **赤い風船を yellow と取り違える**。使わない |
| `camp_mix.pt` | sim 1000 + 実写 161 | 0.3 | — | sim 寄り。sim_eval の F1 は最良 (0.47) |

`cfg` は `width=16 / input_size=256`。`conf_thresh` は **`camp_real2` だけ 0.4**、
他は 0.3 (チェックポイントに格納されているので、`conf_thresh` パラメータを
指定しなければ自動でその値が使われる)。

## 2026-10-08: 既定を `balloon_F320_20261007` にした

umiusi_sim の配備物 `deploy/detector_20261007/`（学習の詳細は umiusi_sim `docs/env_filter_study.md`）。10/03 のラベル付き画像は学習に入れていない。
ONNX 版（`_320.onnx`）はここに置いていない — 入力名が `input` で、ONNX 経路（autonomy #42 / umiusi_sim #5）の読み込みは `x` を渡すため。

umiusi_sim の評価（F1、conf 0.3）: 10/03 未見 run 151920 0.10 → 0.53（黄 0 → 56/99）、144510（遠く淡い青）0.29 → 0.16、
風船なし 55 枚の誤検出 1 → 8、JAMSTEC + 他チーム 70 枚 0.47 → 0.46。**0.40 では濁りの中の風船がほぼ全部落ちる**ので運用は 0.30。

10/03 の 15:19 の映像（赤い風船は無い。赤の検出は全部、風船の下の重りの誤検出）を 320x240・6 Hz 相当で見た、色ごとの最長の連続:

| 閾値 | 赤（誤検出） | 黄 | 青 |
|---:|---|---|---|
| 0.30 | 112 枚 / 36 連続 | 275 枚 / 58 連続 | 175 枚 / 27 連続 |
| 0.45 | 14 枚 / 5 連続 | 9 枚 | 0 |
| **0.50** | **2 枚 / 1 連続** | 1 枚 | 0 |

FSM（umiusi_perception 0009a5d）は、同じ色の検出が 3 フレーム続くと突進に入る（1〜2 フレームなら動かない。手元で確認）。
そのため赤だけ `min_confidence_red: 0.50`。本物の赤い風船の信頼度は 10/03 の映像では分からない（赤い風船が無かった）。

## 2026-10-03: 既定を `balloon_F320_20261003` にした

umiusi_sim `chore/comment-diet` 0009a5d の `examples/balloon_detector/` と同一 (ONNX 版と学習の詳細はそちら、
`docs/otherteam_balloon_data.md`)。入力サイズはチェックポイントに入っているので perception_node の設定変更は要らない。

10/01 の映像 (赤 36 / 風船なし 114) での比較。**この映像は F320 / F256 の学習データに含まれるので甘い**:

| モデル | min_confidence | 赤 (/36) | 風船なしで検出が出たフレーム (/114) |
|---|---:|---:|---:|
| F320 | 0.30 (チェックポイント) | 36 | 11 (うち赤 10) |
| **F320** | **0.40 (既定)** | **34** | **0** |
| F320 | 0.50 | 18 | 0 |
| F256 | 0.40 | 33 | 0 |
| camp_real | 0.45 | 32 | 13 |

推論時間 (開発 PC の CPU、1 スレッド): F320 25 ms / F256 16 ms / camp_real 12 ms。Pi は数倍遅い。
F256 への切り替え:

```bash
ros2 launch umiusi_autonomy core_autonomy.launch.py \
  model_path:=$(ros2 pkg prefix umiusi_autonomy)/share/umiusi_autonomy/models/detector/balloon_F256_20261003.pt
```

## 2026-10-01: 既定を `camp_real` に戻した

10/01 のプールの前カメラ映像 (run1、199 枚を目視でラベル付け) で、`camp_real2` @0.4 は
**画面中央に大きく映った赤い風船を 0/36** しか取れず、ほぼ全部 yellow と出した。
[assumed] 継続学習のデータが黄 26 箱 + 背景だけで、赤のクラスが崩れた。val の F1 0.80 は赤をほとんど含まない。

`camp_real` (前処理は配備どおり) で `min_confidence` を振った結果 (風船なしは 114 枚中、検出が出たフレーム数):

| min_confidence | 赤 (/36) | 壁の黄色いシール | それ以外の誤検出 |
|---:|---:|---:|---:|
| 0.40 | 35 | 9 | 33 (うち赤 1) |
| **0.45 (既定)** | **32** | 4 | 9 (全部黄) |
| 0.50 | 24 | 4 | 1 |

黄・青の風船の実写は 10/01 に無く、未評価。壁際で黄の誤検出が多いときは現場で 0.5 に上げる。
詳細は autonomy `docs/known_issues.md` B-24。

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

既定は `balloon_F320_20261003.pt`。切り替えは launch 引数で:

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
