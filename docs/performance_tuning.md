# 実機の性能チューニング

Raspberry Pi 4 Model B (4 コア) 実機での実測にもとづく。数値は全て `alexandrite` 実機で測定。
測定方法は `tools/bench_rates.py`、スタックの起動は `tools/umiusi_stack.sh` を使うこと
(理由は末尾「測定の落とし穴」)。

## 要点 (先に結論)

| やること | 効果 | コスト |
|---|---|---|
| **torch のスレッドを 1 に固定** | 認識 5.32 → **6.34 Hz** (+19%)、CPU −12 pt | なし。既に launch に組込済 |
| **カメラブリッジを HW デコードに** | CPU 102% → **33〜43%** | なし。既定 |
| カメラブリッジを共有メモリ経路に (`camera_source:=shm`) | デコードが消える。x86 でブリッジ 25% → 2.5% (Pi は `[未検証]`) | control 側の設定を切り替える (opt-in、下記) |
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
`read()` を叩きすぎて別の意味で落ちる。(この表は取得をタイマで回していたころの実測。
今は取得スレッドが全フレームを読み、`max_rate_hz` は publish を時間で間引くだけ — 下記「取得スレッド」)

**供給を絞るのは `max_fps`（既定 15、launch 引数 `camera_max_fps`）。** デコード直後に
`videorate drop-only=true max-rate=<fps>` で間引くので、タイマとはビートしない。間引いた後の
色変換・Python への受け渡し・publish・perception の受信が全部減る。カメラの `framerate`
（`cameras.yaml`、2026-10 時点で 30 fps）は UI の映像と共有しているので下げられない。
`[未検証]` Pi 4 での CPU の実測はまだ（2026-10-03 に「ブリッジが CPU に張り付く」報告を受けて導入）。

> `auto_rate` (消費レートへの AIMD 追従) を実装してあるが**既定は無効**。単純に
> 「供給 = 消費」に追従させると、起動直後の低消費に引きずられて供給が落ち、
> 消費もそれ以上出せなくなるデススパイラルに陥る。AIMD で自己回復するようにしたが、
> 上記のフレーム取りこぼしの問題が残るため実験扱い。

### 共有メモリ経路 (`camera_source:=shm`、opt-in)

既定の経路は Pi の中で「エンコード → RTSP → 即デコード」している。shm 経路はカメラの
パイプラインを `tee` で分け、エンコード前の映像を縮小・間引きして共有メモリに出し、
ブリッジはそれを読むだけにする (デコードしない)。

```
libcamerasrc 1280x720@30 ─ tee ─ videoconvert ! v4l2h264enc ! … ! rtspclientsink (cam1 = UI、既定と同じ)
                                └ queue leaky=downstream max-size-buffers=1 ! videorate max-rate=15
                                  ! videoconvertscale ! BGR 320x240 ! identity drop-allocation=true
                                  ! shmsink socket-path=/tmp/umiusi_cam1.sock
camera_bridge_node: shmsrc socket-path=/tmp/umiusi_cam1.sock ! BGR 320x240 ! appsink
```

- 書き手: `sinsei_umiusi_control` の `params/cameras_shm.yaml` (既定の `cameras.yaml` は変えていない)
- 読み手: `camera_bridge_node` の `shm_socket` (空 = 従来の RTSP)。launch 引数は `camera_source:=rtsp|shm`
  (`core_autonomy` / `autonomy` / `scenario` / `bringup`)。`umiusi_stack.sh` は `UMIUSI_CAMERA_SOURCE=shm`
- 間引き (15 fps) と縮小はカメラ側でやる。shm のときブリッジの `max_fps` / `hw_decode` / `latency_ms` は使わない
- 分岐側は software (`videoconvertscale`)。`v4l2convert` は使えないと negotiation に失敗して
  **UI の映像ごと**落ちるので入れていない

起動:

```bash
ros2 launch sinsei_umiusi_control main.yaml \
  cameras_param_file:=$(ros2 pkg prefix sinsei_umiusi_control)/share/sinsei_umiusi_control/params/cameras_shm.yaml
ros2 launch umiusi_autonomy core_autonomy.launch.py camera_source:=shm
# まとめて: ros2 launch umiusi_autonomy bringup.launch.py camera_source:=shm
#           (cameras_param_file を渡さなければ control の cameras_shm.yaml を使う)
# スクリプト: UMIUSI_CAMERA_SOURCE=shm tools/umiusi_stack.sh start
```

手元 (x86、`videotestsrc` 1280x720@30 + x264enc、2026-10-05) の実測。% は 1 コア比:

| | カメラ (gst_camera_node) | ブリッジ | RTSP サーバ | 計 | `/front_cam/image_raw` |
|---|---:|---:|---:|---:|---:|
| RTSP (既定) | 61% | 25% | 3.7% | 90% | 13.3〜13.6 Hz |
| shm | 67% | **2.5%** | 1.7% | **71%** | **15.0 Hz** |

- x86 にはデコーダが無く、ブリッジは FFMPEG の software デコードで比べている。Pi の HW 経路
  (33〜43%) との比は Pi で測ること (`field_card.md` の手順)
- 分岐側のコストは x86 で +5 pt。Pi の software 縮小がどれだけ食うかは `[未検証]`
- 確認済み (手元): ブリッジを起動しない / 後から起動 / SIGSTOP / SIGKILL のどれでも RTSP 側は 30 fps のまま。
  ブリッジは `reconnect_sec` ごとに開き直し、待機中の CPU は 0.1%。カメラ側の再起動・異常終了
  (古い socket が残ると shmsink は `<path>.0` に作る。ブリッジは最も新しい socket を選ぶ) からも戻る
- ブリッジが止まっていた後は、止まっていた時間ほど 30 Hz で出てから 15 Hz に戻る (`videorate` の追いつき)
- 書き手が生きたまま詰まる (SIGSTOP) と shmsrc の open / read が返らない。下記「取得スレッド」で
  ノードは固まらない。手元 (2026-10-08): 詰まっている間も `ros2 param get` は返り、ERROR が出て、
  SIGCONT 後すぐ 15.0 Hz。書き手を SIGKILL した間は 3 s ごとに開き直して CPU 0.2%
- RTSP サーバが落ちるとカメラのパイプラインごと止まる (既定の経路と同じ。shm も止まる)

### 取得スレッド (RTSP / shm 共通)

`cv2.VideoCapture` の open / read はブロッキングで、OpenCV 4.6 の GStreamer 経路では
`CAP_PROP_OPEN_TIMEOUT_MSEC` / `CAP_PROP_READ_TIMEOUT_MSEC` が使えない (`can't set property` で open ごと失敗)。
そこで open / read は取得スレッドで回し、executor 側は最新フレームを publish するだけにしている。

- 取得スレッド: 開く → 読み続ける → 読めなくなったら `reconnect_sec` (既定 3 s) 待って開き直す
- open / read が `stall_timeout_sec` (既定 5 s) 返らないと ERROR (10 s に 1 回) を出し、取得スレッドを作り直す。
  止まったスレッドは殺せないので残し、後で戻っても結果は捨てる。残してよい本数は `max_stalled_workers` (既定 2)。
  上限に達したら作り直さず、戻るのを待つ
- 終了時は取得スレッドを最大 2 s 待ち、止まったままなら残して終了する

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
