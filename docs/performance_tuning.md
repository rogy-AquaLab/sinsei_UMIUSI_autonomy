# 実機の性能チューニング

Raspberry Pi 4 Model B (4 コア) 実機での実測にもとづく。数値は全て `alexandrite` 実機で測定。
測定方法は `tools/bench_rates.py`、スタックの起動は `tools/umiusi_stack.sh` を使うこと
(理由は末尾「測定の落とし穴」)。

## 要点 (先に結論)

| やること | 効果 | コスト |
|---|---|---|
| **torch のスレッドを 1 に固定** | 認識 5.32 → **6.34 Hz** (+19%)、CPU −12 pt | なし。既に launch に組込済 |
| **カメラブリッジを HW デコードに** | CPU 102% → **33〜43%** | なし。既定 |
| **カメラの解像度/fps を下げる** | `gst_camera_node` 32.7% → **12.8%**、全体アイドル 19% → 52% | 画質 |
| **UI (rosbridge) を止める** | 約 22% のコアを解放 | UI が使えない |
| ブリッジ側でレート制限する | **逆効果** (下記) | — |
| `input_size` を 256 → 192 | 認識 5.6 → 10.4 Hz | **F1 0.69 → 0.55** (非推奨) |

## 1. torch のスレッド数 — 最重要かつ無料

推論そのものの 1 フレーム時間 (ROS を介さない純粋な計測、`tools/infer_bench.py` 相当):

| 条件 | threads=1 | threads=2 | threads=4 |
|---|---:|---:|---:|
| **実機スタック稼働中** | **113.9 ms (8.78 Hz)** | 129.7 ms (7.71 Hz) | 142.0 ms (7.04 Hz) |
| 単独 (他ノードなし) | 60.0 ms (16.7 Hz) | 56.8 ms (17.6 Hz) | **50.8 ms (19.7 Hz)** |

**単独ならスレッドが多いほど速いが、他ノードと CPU を奪い合うと逆転する。**
検出器 (TinyBalloonNet width=16) は小さく、スレッド同期のオーバーヘッドが並列化の利得を
上回るため。実機は常に control/カメラ/BT と同居するので **1 スレッド固定が正解**。

エンドツーエンドでも確認済み (UI なし・供給 10 Hz):

| | 供給 | 認識 | CPU 使用 |
|---|---:|---:|---:|
| `OMP_NUM_THREADS=4` | 6.78 Hz | 5.32 Hz | 67.8% |
| **`OMP_NUM_THREADS=1`** | 6.76 Hz | **6.34 Hz** | **55.7%** |

`core_autonomy.launch.py` の perception_node に `additional_env` で設定済み。
手動起動するときは `OMP_NUM_THREADS=1 MKL_NUM_THREADS=1` を付けること。

## 2. カメラブリッジ

デコードだけでなく**色変換と縮小もハードウェアに逃がす**。

| パイプライン | CPU |
|---|---:|
| `videoconvert ! videoscale` (software) | **102%** (CPU 律速でレートも 15 → 11.6 Hz に低下) |
| **`v4l2h264dec ! v4l2convert`** (hardware) | **33〜43%** |

`camera_bridge_node` の既定は HW 経路で、開けない環境では software に自動フォールバックする。

### レート制限はブリッジのタイマではなく GStreamer の中で (`max_fps`)

ブリッジの `max_rate_hz` で絞ると**フレームを取りこぼして逆効果**になる:

| ブリッジ設定 | 実供給 | 認識 |
|---|---:|---:|
| **制限なし (既定)** | **13.96 Hz** | **6.08 Hz** |
| 12 Hz 制限 | 5.05 Hz | 5.03 Hz |
| 10 Hz 制限 | 6.70 Hz | 4.08 Hz |

タイマ周期を目標レートにするとカメラのフレーム到着とビートし、1 ms まで速めると
`read()` を叩きすぎて別の意味で落ちる。

**供給を絞るのは `max_fps`（既定 15、launch 引数 `camera_max_fps`）。** デコード直後に
`videorate drop-only=true max-rate=<fps>` で間引くので、タイマとはビートしない。間引いた後の
色変換・Python への受け渡し・publish・perception の受信が全部減る。カメラの `framerate`
（`cameras.yaml`、2026-10 時点で 30 fps）は UI の映像と共有しているので下げられない。
`[未検証]` Pi 4 での CPU の実測はまだ（2026-10-03 に「ブリッジが CPU に張り付く」報告を受けて導入）。

> `auto_rate` (消費レートへの AIMD 追従) を実装してあるが**既定は無効**。単純に
> 「供給 = 消費」に追従させると、起動直後の低消費に引きずられて供給が落ち、
> 消費もそれ以上出せなくなるデススパイラルに陥る。AIMD で自己回復するようにしたが、
> 上記のフレーム取りこぼしの問題が残るため実験扱い。

## 3. 供給レートと認識レートの関係

供給しすぎても足りなくても落ちる (実機スタック稼働下、`input_size=256`):

| 実供給 | 認識 |
|---:|---:|
| 4.77 Hz | 3.53 Hz |
| 5.95 Hz | **4.82 Hz** |
| 6.70 Hz | 4.08 Hz |
| 8.71 Hz | 4.46 Hz |
| 13.96 Hz | **6.08 Hz** (1 スレッド設定時) |

供給過多だと受信・デシリアライズに CPU を取られる。**供給（`max_fps`）は
目標認識レートの 1.5〜2 倍程度**が目安。

## 4. `input_size` — 速度と精度の取引 (非推奨)

ラベル付き val セットでの評価:

| `input_size` | F1 | precision | recall | 認識周期 |
|---:|---:|---:|---:|---:|
| **256** (既定) | **0.69** | 0.66 | 0.72 | 5.6–8.3 Hz |
| 192 | 0.55 (−20%) | 0.58 | 0.54 | **10.4 Hz** |
| 160 | 0.42 (−39%) | 0.46 | 0.39 | — |

10 Hz には届くが **recall が 0.72 → 0.54** で、風船を 4 分の 1 余計に見逃す。
先に 1〜3 の施策を尽くすこと。

`conf_thresh` は**速度に効かない** (0.3 → 0.5 で検出数 31 → 0 になっても 4.88 → 4.58 Hz)。
CNN の推論コストは画像の中身に依らないため。誤検出を減らす目的でのみ使う。

## 4b. ONNX Runtime バックエンド — **Pi で未測定。既定は torch のまま**（2026-10-03 更新）

仕様: `mujoco_ws/ai/spec_perception_onnx.md`。

```bash
ros2 launch umiusi_autonomy core_autonomy.launch.py backend:=onnx   # 起動時にだけ効く
```

- 同じ重みを onnxruntime で回す。出力は torch と一致する（hm / wh の差 4e-6 以下、Detection は一致。
  umiusi_sim `tests/test_learned_onnx.py`、autonomy `test/test_bundled_onnx.py`）
- **.onnx は .pt の隣に同梱**（`models/detector/<重み名>_<input_size>.onnx`、F320 @320 と F256 @256）。
  読み込み時に torch と出力を比べ、合わなければ（.pt だけ差し替えた等）使わない
- 同梱が無い組み合わせ（`input_size` を変えた / 他の .pt）は初回に書き出して `~/.cache/umiusi_perception/` に置く。
  **書き出しには `onnx` パッケージが要る**。Pi に無ければ下の torch 戻りになる。
  他の組み合わせを使うなら PC で umiusi_sim `tools/export_detector_onnx.py` を回して .pt の隣に置く
- **onnx が使えなければ ERROR `onnx backend unavailable (...); falling back to torch` を出して torch で動き続ける**
  （onnxruntime が無い / wheel が古い / 書き出し失敗）。起動ログの `backend=` で実際に使われた方が、
  `onnx='...'` でどの .onnx かが分かる
- `ros2 param set ... backend` は**拒否される**（読み込み後に変えても効かないため）
- 前処理（リサイズ）は 2026-10-03 に速くした（出力はビット単位で同じ）。torch / onnx の両方に効く。
  umiusi_perception の wheel を入れ直さないと効かない

x86（開発 PC）・1 スレッド・前カメラと同じ 1280x720、`tools/infer_bench.py` の中央値 (ms):

| モデル | backend | 前処理 | モデル | decode | **1 フレーム** |
|---|---|---:|---:|---:|---:|
| F320 | torch | 4.8 | 12.4 | 0.3 | **18.0** |
| F320 | **onnx** | 4.9 | **10.5** | 0.3 | **15.5**（x1.16） |
| F256 | torch | 4.3 | 7.9 | 0.2 | 13.0 |
| F256 | **onnx** | 4.3 | **5.8** | 0.2 | 11.1（x1.17） |

- 前処理の高速化: 1280x720 で 5.8 → 5.1 ms（繰り返し実行時）。確保し直しが起きる条件では 15.7 → 4.9 ms。
  実機でどちらに近いかは未測定
- perception_node を通した周期（x86、F320、画像を詰めて送った上限）: torch 44.0 Hz → onnx 49.7 Hz
- int8（静的量子化）は x86 で fp32 より遅く（F320 11.3 ms）、検出も変わる（40 枚中 16 枚しか一致しない）。使わない
- **x86 の比は Pi の比ではない**。Pi で測るまで既定は torch のまま

### Pi で測る手順（受け入れ条件）

Pi は PC 経由でネットに出られる前提。**4 つとも、起動ログの 1 行（`detector loaded from ...`）を控える。**

```bash
# 0. 入れる（autonomy と umiusi_sim の両方を feat/perception-onnx-2 に。ビルドはしない）
cd ~/ros2-ws/src/sinsei_UMIUSI_autonomy && git fetch && git checkout feat/perception-onnx-2
cd ~/umiusi_sim && git fetch && git checkout feat/perception-onnx-2
python3 -m pip install --user --break-system-packages --no-deps --no-index ~/umiusi_sim/packages/perception
# numpy を今の版に固定する（onnxruntime が numpy を上げると ROS / torch が壊れる）
python3 -m pip install --user --break-system-packages onnxruntime "numpy==$(python3 -c 'import numpy; print(numpy.__version__)')"
python3 -c "import onnxruntime; print(onnxruntime.__version__)"
# 新しい .onnx は install/ に入っていない（ビルドしないため）。src の .pt を直接指す
M=~/ros2-ws/src/sinsei_UMIUSI_autonomy/umiusi_autonomy/models/detector
```

1. **Pi 単独の推論周期**（スタックを止めて）:
   ```bash
   cd ~/ros2-ws/src/sinsei_UMIUSI_autonomy
   NT=1 python3 tools/infer_bench.py $M/balloon_F320_20261003.pt
   NT=1 python3 tools/infer_bench.py $M/balloon_F256_20261003.pt
   ```
   torch / onnx の 1 フレームと内訳（前処理 / モデル / decode）が出る。内訳が出なければ wheel が古い
2. **core_autonomy.launch.py 稼働中の周期と CPU**（UI なし、STANDBY のまま）。backend ごとに起動し直す:
   ```bash
   ros2 launch umiusi_autonomy core_autonomy.launch.py backend:=onnx model_path:=$M/balloon_F320_20261003.pt
   # 別の窓で。STANDBY では推論しないので止める設定を外す
   ros2 param set /perception_node infer_only_in_auto false
   python3 tools/bench_rates.py --duration 30 /front_cam/image_raw /perception_node/detections
   top -bn1 | head -20        # perception_node（python3）の %CPU
   ```
   `backend:=torch` と、F256 の torch / onnx でも同じことをする（計 4 回）
3. **onnxruntime が無いとき torch に戻るか**（パッケージを消さずに import だけ塞ぐ）:
   ```bash
   mkdir -p /tmp/noort/onnxruntime && echo 'raise ImportError("onnxruntime を塞いだ")' > /tmp/noort/onnxruntime/__init__.py
   PYTHONPATH=/tmp/noort:$PYTHONPATH ros2 launch umiusi_autonomy core_autonomy.launch.py backend:=onnx model_path:=$M/balloon_F320_20261003.pt
   ```
   期待: ERROR `onnx backend unavailable (ImportError: ...); falling back to torch` → `backend=torch` の行 →
   `ros2 param set /perception_node infer_only_in_auto false` の後に `/perception_node/detections` が出続ける。
   本当に外して確かめるなら `python3 -m pip uninstall -y onnxruntime`（戻すときは 0. の install）

判定: **onnx・F320 の周期が torch・F256 以上なら**、競技構成の推奨として `backend:=onnx` をここに書く。
既定（config/autonomy.yaml と launch の `backend`）はその後に変える。

## 5. 構成別の実測サマリ

| 構成 | `/state/imu` | 姿勢制御 | 画像 | 認識 | アイドル |
|---|---:|---:|---:|---:|---:|
| control のみ (カメラ無し) | 50.0 Hz | — | — | — | 高 |
| control + カメラ + ブリッジ + perception | 50.0 Hz | — | 13.96 Hz | 6.08 Hz | 41.5% |
| フルスタック 22 ノード (姿勢制御 + UI + BT) | 50.1 Hz | 33.8 Hz | 12.3 Hz | 5.17 Hz | 3.6% |

**C++ の `ros2_control` は CPU が飽和しても 50 Hz を維持する。Python 系
(姿勢制御・perception・画像受信) が先に劣化する。** 温度は 50°C 程度でスロットルは
発生せず、**制約は CPU であって熱ではない**。

## 6. 測定の落とし穴 (重要)

* `timeout N ros2 launch ... &` で起動すると**計測の途中で寿命が切れて上流が消える**。
  そうなると「入力レートより認識周期が高い」といった辻褄の合わない値が出る。
  起動は `tools/umiusi_stack.sh` に寄せること。
* `ros2 topic hz` の出力をパースする方式は、publisher が居ないのか単に遅いのか
  区別できない。`tools/bench_rates.py` は **publisher 数も併せて報告する**ので、
  0 Hz の原因を取り違えない。
* `ps -eo pcpu` は**プロセス生涯の平均**を返す。瞬間値が欲しいときは `top -bn1` を使う。
  (これを取り違えて「perception は 1 コアしか使っていない」と誤読しかけた。)
* 室内の雑然としたシーンは誤検出が多く出るが、**検出数は推論コストに影響しない**。
  ただし供給レートは変わるので、シーンを変えて比較するときは供給を揃えること。

---

## 7. 前カメラ (CSI) と下カメラ (USB) の比較 — 2026-08-20 実測

前方 = **Raspberry Pi Camera Module 3 NoIR (`imx708_noir`)**、下方 = USB H264 カメラ。
UI では `cam1` = 前、`cam2` = 下 に対応する。

| 経路 | 解像度 | 画像レート | 認識周期 | CPU 使用 |
|---|---|---:|---:|---:|
| **前カメラ (CSI / imx708)** | 1280x720@15 | **15.36 Hz** | **7.70 Hz** | 66.2% |
| 下カメラ (USB) | 800x600@15 | 13.96 Hz | 4.73 Hz | 55.3% |

**前カメラのほうが速い。** CSI は unicam → ハードウェア ISP を通るため、
`libcamerasrc` の出力がそのままエンコーダに渡せて軽い (パイプライン単体で **CPU 14.8%**)。
perception には前カメラを使うこと (下カメラは位置制御用)。

> 前カメラを動かすには `GST_PLUGIN_PATH` の設定が要る (`known_issues.md` B-2b)。

## 8. ロガーを動かしたときの性能低下 — ほぼゼロ

両カメラを同時に録画した状態で計測 (`tools/record_camera.sh --both --raw`):

| | 画像 | 認識 | CPU 使用 |
|---|---:|---:|---:|
| ロガー無し | 15.31 Hz | 7.56 Hz | 66.0% |
| **両カメラ録画中** | **15.23 Hz** | **7.58 Hz** | **68.4%** |

**低下は測定誤差の範囲**で、CPU は +2.4 ポイントのみ。H264 をデコードせず
そのまま書き出すため、録画は実質タダで回せる。競技本番でも常時録画してよい。

記録されたファイル (20 秒):

| ファイル | サイズ | 内容 |
|---|---:|---|
| `cam1.h264` | 10.2 MB | h264 1280x720 (前) |
| `cam2.h264` | 35.7 MB | h264 800x600 (下) |

下カメラのほうが大きいのは、USB カメラ内蔵エンコーダのビットレートが高いため
(前カメラは `video_bitrate=3000000` = 3 Mbps 指定)。容量が問題なら
`cameras_deploy.yaml` の bitrate を下げる。

---

## navigator の setpoint 経路を詰める (2026-09-22)

`tools/navigator_sim.py` で閉ループを回した結果。**MuJoCo は使っていない** —
`configs/umiusi.yaml` の `thrust_axes` が実機と yaw が鏡像のままなので、そこで詰めた
旋回ゲインは実機で逆に効く。代わりに**配備バンドルの幾何 + 実機 bag から測った動特性**
（roll/pitch の復元、起動の死に時間 2.7 s。known_issues B-18）を真値にしている。

**較正前の契約値を使っているので絶対値は信用しない。ゲインの相対比較に使う。**

### `cmd_target_yaw_lead_max` は 1.05 → **0.35 rad (20°) を推奨**

| `lead_max` | 旋回レート | 方位誤差 p95 | 整定後の誤差 |
|---:|---:|---:|---:|
| 0.35 rad (20°) | 16.07 deg/s | **20.1°** | 6.7° |
| 0.52 rad (30°) | 16.39 | 25.1° | 6.7° |
| **1.05 rad (60°・既定)** | 16.97 | 29.2° | 6.7° |
| 1.57 rad (90°) | 16.97 | 29.2° | 6.7° |

**旋回レートは 5% しか落ちず、追従誤差は 3 分の 2 になる。** 整定後の誤差は同じ。
1.05 以上は頭打ち（その速さでは先行量が上限に当たらない）。
**180° の罠 (B-14) に対する余裕も増える**ので、下げる方向に迷う理由が無い。

### 起動の死に時間は **arm 直後に 1 回だけ**効く

30 秒その場旋回させたときの旋回量は、死に時間の有無で**変わらない**（どちらも 509°）。
効くのは追従誤差だけ: **p95 6.7° → 29.2°**。目標が先行して待ち、スラスタが回り始めたら
追いつくため。逆に**指令が 3 秒しか続かないと 2.7 秒ぶんが食われる**（旋回 103° → 68°）。

  * **arm してから 3 秒は制御の権限が無いと思うこと。** 浅場でも本番でも同じ
  * **FSM の短い補正パルスは効きにくい。** ただし姿勢制御器が方位を保持し続けるので
    スラスタは回りっぱなしになり、**パルスのたびに払い直すことは無い**
    （0.5〜2 秒のパルス列で実効率 98〜100%）

### 3 基運用では cap が**そのまま前進速度**になる

前進しながら旋回する想定（`--scenario approach --live 0,1,1,1`）:

| cap | 前進速度 | 方位誤差 p95 | duty 飽和 |
|---:|---:|---:|---:|
| 0.25 | 0.11 m/s | 33.4° | **100%** |
| 0.30 | 0.14 m/s | 31.3° | 100% |
| 0.40 | **0.20 m/s** | 28.6° | 100% |

**全域で duty が上限に張り付く**ので、cap を上げたぶんがそのまま速度になる
（4 基なら cap 0.25 で 0.17 m/s）。旋回レートは cap に依らない。
**風船へ寄る所要時間を半分にしたいなら cap 0.4。**

### 再現

```bash
python3 tools/navigator_sim.py --sweep lead_max              # 上の 1 つ目の表
python3 tools/navigator_sim.py --scenario step --dead-time 0 # 死に時間の効き
python3 tools/navigator_sim.py --scenario approach --live 0,1,1,1 --sweep cap
```

### `kd` は**下げてはいけない**。死に時間はオーバーシュートを増やす

sim 側から「起動の死に時間 2.7 s と `kd=0.35` の微分項は相性が悪いはずなので、実機では
kd を下げる方向で」という見立てが来たので、同じ台で測った。**逆だった。**

方位ステップ（3 秒旋回して止める）のオーバーシュートと整定:

| `kd` | 死に時間あり（実機相当） | | 死に時間なし | |
|---:|---:|---:|---:|---:|
| | 行き過ぎ | 整定 | 行き過ぎ | 整定 |
| 0.00 | **33.3°** | 6.30 s | 5.8° | 1.36 s |
| 0.10 | 19.5° | 3.44 s | 5.3° | 0.60 s |
| 0.20 | 8.7° | 2.54 s | 2.9° | 0.10 s |
| **0.35（現行）** | **1.2°** | **2.00 s** | 0.4° | 0.30 s |
| 0.60 | 0.0° | 2.62 s | 0.0° | 0.82 s |

**死に時間はオーバーシュートを 6 倍にする**（kd=0 で 5.8° → 33.3°）。それを抑えているのが
kd で、下げると悪化する一方。**回転慣性を 0.5〜2 倍に振っても順序は変わらない**
（kd=0 の行き過ぎ 41.3 / 33.3 / 24.9°、kd=0.35 で 0.0 / 1.2 / 4.1°）。

> **ただし kd の最適値そのものは信用しないこと。** この台の**回転慣性と回転抗力は概算**で
> （契約値に無い）、回転抗力は kd と同じ働きをする。**頑健なのは「下げる方向は逆」という
> 向きだけ。** 迷うなら 0.35 → 0.5 の側（慣性 2 倍でも行き過ぎ 4.1° が残るため）。
