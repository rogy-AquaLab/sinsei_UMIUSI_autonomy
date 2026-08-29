# ロギング — 競技後に分析できるように残す

## なぜ rosbag だけでは足りないか

実機カメラは `gst_camera_node` が **RTSP に流すだけで ROS トピックを出さない**
(`gst_camera_node` は GStreamer パイプラインのアダプタで publisher を持たない)。
そのため `ros2 bag record -a` を回しても**映像は 1 フレームも残らない**。

さらに **V4L2 デバイスは二重に開けない**。`gst_camera_node` が `/dev/video4` を
掴んでいる間、別プロセスが同じデバイスを開こうとすると `Device is busy` で失敗する
(実測で確認済み)。したがって録画は **RTSP 側から** 行う必要がある。

## PC に取り出す

`record_run.sh` は最後の run へのリンク `~/runs/latest` を張るので、名前を調べる必要はない。

```bash
# 最新の 1 本だけ (WSL / Linux / Windows の scp どれでも)
scp -r pi@<機体>.local:runs/latest/ .

# 全部を差分同期する (2 回目以降が速い。WSL から叩けば ROS は要らない)
rsync -avz pi@<機体>.local:~/runs/ ./runs/
```

> **WSL で ROS が繋がらなくても scp / rsync は普通に使える。** DDS が通らないのは
> WSL2 が NAT だからで、SSH は outbound なので影響しない。解析だけ PC でやるならこれで足りる。

取り出したら:

```bash
ros2 bag info runs/latest/bag
ros2 bag play runs/latest/bag        # PC 側で再生して rqt / PlotJuggler で見る
```

## 方針: 映像は H264 のまま、それ以外は rosbag

| 対象 | 手段 | コスト |
|---|---|---|
| 映像 | RTSP から H264 のまま録画 (`tools/record_camera.sh`) | **CPU 15.5%**、1.25 MB/s (800x600@15) |
| 状態・指令・検出 | `ros2 bag record` (映像トピック以外) | 小さい |
| ノードのログ | `~/umiusi_logs/` | 小さい |

デコードも再エンコードもしないので、Pi の CPU をほとんど食わない。

## 使い方

```bash
# 映像: 前後カメラを同時に、切り捨てに強い生 H264 で (Ctrl-C で停止)
./tools/record_camera.sh --both --raw

# それ以外: rosbag。**-a は使わないこと** (下記)
ros2 bag record -o run_$(date +%Y%m%d-%H%M%S) \
  /state/imu /state/pressure /state/imu_temperature \
  /rl_attitude_node/current_setpoint /rl_attitude_node/depth \
  /rl_attitude_node/depth_mode \
  ...  # 以下略 — 実際の一覧は tools/record_run.sh を見ること
```

**録るトピックの一覧は `tools/record_run.sh` が正**（22 トピック、`--vision` で +2）。ここに手で写した一覧は
すぐ古くなるので置かない。上は雰囲気をつかむための抜粋で、`/rl_attitude_node/current_setpoint`
`/rl_attitude_node/depth` `/rl_attitude_node/depth_mode` `/state/pressure`
`/state/imu_temperature` を含む全量はスクリプト側で管理している。手で `ros2 bag record` を
打つ場面でも、まず `record_run.sh` の一覧をコピーすること。

## `ros2 bag record -a` は使わない — 実測

`-a` は `/front_cam/image_raw` (生 `sensor_msgs/Image`) まで録ってしまう。実機で計測した差:

| | 認識周期 | CPU 使用 | 容量 |
|---|---:|---:|---:|
| bag 無し | 7.74 Hz | 63.2% | — |
| **トピックを絞る** | **7.66 Hz** | **67.6%** (+4.4 pt) | **184 MB/時** |
| `-a` (全トピック) | 6.84 Hz (**−12%**) | 93.2% (**+30 pt**) | **39 GB/時** |

**`-a` は容量が 200 倍、CPU を 30 ポイント食い、perception が 12% 落ちる。**
一方で**絞れば perception への影響は誤差の範囲**に収まる。

`-a` が重い理由は `/front_cam/image_raw` ただ 1 つで、他のトピック
(`/state/*`, `/cmd/*`, `/tf`, `/joint_states`, `/rosout`) はすべて小さい。
映像は上の `record_camera.sh` で H264 のまま録るほうが、**CPU も容量も桁違いに安い**
(録画の追加コストは CPU +2.4 pt)。

> どうしても bag に映像を入れたいときだけ `camera_bridge_node` の
> `publish_compressed:=true` を使い、`/front_cam/image_raw/compressed` を録る。
> 生 Image (`/front_cam/image_raw`) は絶対に bag に入れない。

出力先は `~/recordings/<開始時刻>/` で、`meta.txt` に開始時刻 (ISO と UNIX 秒)、
RTSP URL、セグメント長、ホスト名が入る。**rosbag と同じ Pi の時計なので、
この開始時刻を基準に映像と bag を突き合わせられる。**

## mp4 セグメント vs 生 H264 — 実測にもとづく使い分け

| | mp4 セグメント (既定) | **生 H264 (`--raw`)** |
|---|---|---|
| 途中の電源断・`kill -9` | **最後のセグメントが壊れる** (`moov atom not found`) | **壊れない。そのまま再生できる** |
| 失う長さ | 最大 1 セグメント分 (既定 30 秒) | ほぼゼロ |
| 扱いやすさ | そのまま再生・シーク可 | mp4 化が一手間 |
| ファイル分割 | 自動 | 単一ファイル |

**競技本番など「確実に残す」ことが最優先なら `--raw` を使うこと。** 実測で
`kill -9` した後でも `ffprobe` が h264 / 800x600 と認識でき、内容を失わなかった。

> 生 H264 で保存するときは **Annex-B バイトストリーム**にする必要がある。
> `h264parse` の既定出力は AVC (長さ前置) で、そのまま `filesink` に書くと
> start code が無く再生できない (実際に踏んだ)。`record_camera.sh` は
> `video/x-h264,stream-format=byte-stream,alignment=au` を挟んで対処済み。

### mp4 化 (再エンコードなし)

```bash
ffmpeg -nostdin -r 15 -i cam.h264 -c copy cam.mp4
```

`-c copy` なので画質劣化なし・一瞬で終わる (28 MB → 28 MB)。

> **`cam1_r1.h264` のような `_rN` 付きのファイルがあれば、それも忘れずに変換すること。**
> `record_camera.sh` は RTSP が途切れて gst が落ちたら自動で録画を再開し、そのたび `_rN`
> (N = 再起動回数) を付けた新しいファイルを作る (同じ名前で開き直すと `filesink` が先頭から
> 上書きしてそれまでの録画が消えるため)。1 本だけ変換すると再起動以降を取りこぼす。
`-nostdin` を付けないと ffmpeg が標準入力を食ってスクリプトが壊れる。

## mp4 セグメントを使うときの注意

`splitmuxsink` は **最後のセグメントを閉じるのに EOS が要る**。
`kill -9` や電源断では `moov atom` が書かれず、そのセグメントは失われる。

* 停止は必ず **Ctrl-C (SIGINT)**。`record_camera.sh` は SIGINT を受けて EOS を流し、
  finalize を待ってから終了する
* セグメントは短めに (既定 30 秒)。失う最大長がそのまま損失になる
* 電源断が心配なら `--raw`

## 容量の目安

800x600@15 で **約 1.25 MB/s = 4.3 GB/時**。実機の空きは 200 GB 以上あるので
競技当日を通しで録っても問題ない。解像度と bitrate を下げればさらに減る
(`cameras_deploy.yaml` の `video_bitrate`)。

## 映像を ROS 側にも残したい場合

`camera_bridge_node` に `publish_compressed:=true` を渡すと
`<image_topic>/compressed` に JPEG (`sensor_msgs/CompressedImage`) を出す。
これは rosbag にそのまま入るので、**bag だけで映像も含めて完結**させたいときに使う。

```bash
ros2 run umiusi_autonomy camera_bridge_node --ros-args \
    -p publish_compressed:=true -p jpeg_quality:=80
```

ただし JPEG エンコードの CPU を新たに払うことになる。**CPU に余裕が無い実機本番では
RTSP 直録 (上記) のほうが安い。** 生 `sensor_msgs/Image` を bag に入れるのは
320x240 でも 3.5 MB/s あるので勧めない。

### カメラで位置を固定する準備として録る (`--vision`)

視覚で位置を推定するなら、**H264 の映像だけでは足りない**。フレームと bag のイベントを
対応づける手段が `meta.txt` の開始壁時計しかなく、`rtspsrc` のジッタバッファ (100 ms) と
再接続 (`_rN`) があるので単純な積算では合わないため (known_issues A-16)。

```bash
# スタック側: 圧縮画像も出す
ros2 launch umiusi_autonomy core_autonomy.launch.py record_vision:=true
# 記録側: それを bag に入れる
./tools/record_run.sh --vision --name pool-vision
```

`record_vision:=true` は `<image_topic>/compressed` を **2 Hz に間引いて** publish する
(`compressed_max_rate_hz`)。全フレームは RTSP 直録が持っているので、bag 側は
**映像と bag を突き合わせる基準**と「そのとき何が見えていたか」があれば足りる。
間引きは `cv2.imencode` を呼ぶ前に効くので、**払う CPU も 2 Hz ぶん**で済む。

> `/front_cam/camera_info` も録る一覧に入れてあるが、**いまはどのノードも publish しない**。
> 内部パラメータが未較正で、しかも 16:9 → 4:3 の強制リサイズで方位角が系統的にずれている
> (**known_issues A-14 — 位置固定を作る前にここを潰すこと**)。購読チェックがこれを毎回
> 「まだ publish されていない」と出すのは想定どおりで、較正が入ったら消える。

### 対地速度をフローで測る準備 (`--flow`) — **ROS トピックは要らない**

**下向きカメラ (cam2) を ROS に流す必要はありません。** 映像は RTSP 直録の H264 が
**全フレーム (15 fps、800x600) 持っています** — ブリッジ経由の 320x240 より条件が良く、
デコードも JPEG エンコードも払いません。RTSP は**複数クライアントが同時に読めます**
(前カメラは既に `camera_bridge_node` と `record_camera.sh` が同時に読んでいる)。

足りないのは映像ではなく **フレームの時刻**だけです。`--raw` の生 H264 は Annex-B
バイトストリームで**コンテナが無く時刻情報がゼロ**なので、「15 fps 一定」を仮定するしか
ありません。フローは `速度 = フロー[px/s] × h / fx` なので、フレーム間隔がずれると
速度がそのままずれます。

```bash
./tools/record_run.sh --flow --name pool-flow
```

- **下カメラだけ mp4 セグメント**にします (`splitmuxsink` が PTS を保持)。前カメラは
  `--raw` のままなので、電源断で失うのは下カメラの最大 30 秒ぶんだけです。
- **`cam2_controls.txt` に露光/ゲインの実値**を残します (`v4l2-ctl --list-ctrls`、読むだけ)。

#### 露光がフローの成否を分ける

高度 1 m・速度 0.17 m/s (cap 0.25 の物理上限) で、下向き 800x600 のフローは **約 118 px/s**。
つまり**ブレ [px] ≒ 118 × 露光時間 [s]**:

| 露光 | ブレ | 判定 |
|---:|---:|---|
| 17 ms | 2 px | 良好 |
| 42 ms | 5 px | 際どい |
| 100 ms (水中の自動露光はここまで伸びる) | **12 px** | **特徴点が流れてフローが出ない** |

**露光を 20 ms 以下に固定してゲインで稼ぐ**のが正解です。フローはブレより粒状ノイズに強い。

```bash
# ⚠ 実機未検証。デバイス番号は cameras_deploy.yaml の usb_camera と揃えること
v4l2-ctl --device=/dev/video4 --set-ctrl=auto_exposure=1            # 1 = manual
v4l2-ctl --device=/dev/video4 --set-ctrl=exposure_time_absolute=200 # 単位 100us -> 20 ms
```

コントロール名はデバイスによって違う (`exposure_auto` / `auto_exposure` など) ので、
**まず `--list-ctrls` で実際の名前を確認**してください。ストリーム中に効かない場合は
スタックを起動する前に設定します。

> **設定できなくても、記録だけは必ず残すこと。** その回の露光が分からないと、フローが
> 出なかったときに**「ブレで出なかったのか、床にテクスチャが無かったのか」を切り分けられず、
> 2 回目も同じ結果になります。**

#### 換算に必要なのは `fx` と `h` ではなく `K = fx/h` ひとつ

`flow = (fx/h) × v` なので、**カメラ較正板も画角の実測も要りません** (A-14 の較正を待つ必要なし)。
**既知距離を一定深度で泳ぎ、フローの積分値と実距離を突き合わせれば `K` が決まります。**
前提は「その run の間、高度がほぼ一定」なので、水圧センサが無ければ定深度で泳ぐ運用にします。

低速側は注意: 定点保持 (0.02 m/s) だとフローは 14 px/s しかなく、15 fps で 1 px/frame を切って
ノイズ床に埋もれます。**推定器は速度に応じて比較の基線を伸ばす** (t と t-3 フレームを比べる等)
設計にしないと、一番欲しい定点保持で効きません。

## bag の metadata が書かれないことがある — 真の原因は SIGINT の継承

`ros2 bag record` を止めたのに **`metadata.yaml` が書かれず**、`ros2 bag info` / `play` が
`Could not find metadata in bag directory` で開けないことがある。

当初これを「rosbag2 の癖」と書いていたが**誤りだった**。真の原因は POSIX のシグナル継承:

* **非対話シェルが `&` で起こした子プロセスは、SIGINT を `SIG_IGN` のまま継承する**
* CPython は SIGINT が `SIG_IGN` のとき既定ハンドラを入れないので、
  **`ros2 bag record` は SIGINT を完全に無視する** → SIGKILL で殺される → metadata なし
* 同じ理由で、シェルスクリプト側の `trap ... INT` も**入口で無視されたシグナルは trap できない**
  ため無効化される (bash の仕様)。録画スクリプトの graceful stop が毎回空振りしていた

**対処済み**: `record_camera.sh` / `record_run.sh` は `set -m` (ジョブ制御) を有効にして
子を独立したプロセスグループで起こし、SIGINT が届くようにした。`umiusi_stack.sh` は
`SIGINT → SIGTERM → SIGKILL` の段階的停止にした。

> **`set -m` はサブシェルの中では効かない。** bash はフォークしたサブシェルで job control を
> 無効化するので、`( ... ) &` の中で起こした子は SIGINT/SIGQUIT を `SIG_IGN` で継承する。
> `record_camera.sh` の監視ループは `( ... ) &` の中で gst を起こすため、**サブシェルの先頭で
> `set -m` を入れ直している**。これを外すと gst に SIGINT が届かず finalize が空振りする
> (いまは GLib が `SIG_IGN` を上書きするので結果的に動くが、依拠してよい性質ではない)。
>
> **フォアグラウンドの `sleep` も同じ罠。** `set -m` 下で `sleep N` を前景で回すと Ctrl-C は
> `sleep` のプロセスグループにしか届かず、親の `trap` が走らない (2 回目でようやく効く)。
> `record_run.sh` の購読確認は `sleep N & wait $!` にしてある。

> **実機での Ctrl-C 動作は未検証。** `set -m` により子が別プロセスグループになるため、
> tty からの Ctrl-C は親にだけ届き、親の trap が子へ転送する形に変わる。次回の実機作業で
> 確認すること。

### それでも開けないときは reindex

**データは失われていない。** MCAP は自己記述形式なので完全に復元できる:

```bash
ros2 bag reindex <bag ディレクトリ>
```

復元例 (実機、149 秒ぶん): `/state/imu` 6576 件、`/perception_node/detections` 993 件、
`/state/thruster_state_all` 6581 件 — すべて読めるようになる。

**走行後に `record_run.sh --fix` を打つ運用**にしておけば、`~/runs` 配下でこの状態に
なっているものをまとめて直せる。保険として残してある。

## 走行 1 回ぶんをまとめて記録する

```bash
./tools/record_run.sh --name pool-01
```

映像 (前後カメラ、H264 そのまま) と rosbag (状態・指令・検出の 22 トピック) を同時に開始し、
Ctrl-C で両方をきれいに閉じる。出力は `~/runs/<日時>-<名前>/` (`UMIUSI_RUN_DIR` で変更可) に
`video/` `bag/` `meta.txt` が揃う。**実機実測で perception への影響なし** (7.74 -> 7.80 Hz)。

そのうち 3 つは **「前進しない」の切り分け専用**に入れてある。8/25 の run はここが無くて
「指令したのに出なかったのか、そもそも指令していないのか」を bag から確定できなかった:

| トピック | これで分かること |
|---|---|
| `/rl_attitude_node/setpoint` | teleop が**送った**目標。`current_setpoint` (ノードが**適用した**目標) と突き合わせると、届いていないのか無視されたのかが分かる |
| `/rosout` | **どのポリシーで走ったか** (`policy loaded … obs 17-D`)、`目標を更新`、`ARMED`。14 次元ポリシーは速度指令を受理表示しつつ黙って捨てるので、これが無いと区別できない (A-15) |
| `/rl_attitude_node/estop` | 武装/解除の履歴。`~/arm` は**サービス**なので topic には出ない |

`bag_check.py` はこの `/rosout` を読んで、**14 次元ポリシーに速度指令が入っている run を
FAIL にする**。プールサイドで気付けるようにするため。

## 最低限これだけは残す (競技当日)

1. **`tools/record_run.sh --name <走行名>`** — 映像と bag を同時に開始する。
   個別に回すより取りこぼしが少ない
2. **走行後に `tools/record_run.sh --fix`** — 停止処理が最後まで走らないことがあるので、
   metadata の復元はここで確実に取り切る (上記「rosbag の metadata が…」参照)
3. **取ったらその場で `python3 tools/bag_check.py <bag>` で検品** — 前後静止 5 s・IMU 化け・
   衝突スパイク・励起カバレッジ・**どのポリシーで走ったか**を見る。撤収してからでは取り直せない
3b. 対地速度をフローで測るつもりなら **`--flow` を付ける**。下カメラの mp4 (フレーム時刻) と
   露光の実値が残る。**露光は先に 20 ms 以下へ固定**しておくこと (上記「対地速度をフローで…」)
4. `~/umiusi_logs/` のノードログ (`tools/umiusi_stack.sh` が自動で残す)
5. 走行前に `tools/bench_rates.py` を 20 秒回して**その日の周期の記録**を取る
