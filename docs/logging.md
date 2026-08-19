# ロギング — 競技後に分析できるように残す

## なぜ rosbag だけでは足りないか

実機カメラは `gst_camera_node` が **RTSP に流すだけで ROS トピックを出さない**
(`gst_camera_node` は GStreamer パイプラインのアダプタで publisher を持たない)。
そのため `ros2 bag record -a` を回しても**映像は 1 フレームも残らない**。

さらに **V4L2 デバイスは二重に開けない**。`gst_camera_node` が `/dev/video4` を
掴んでいる間、別プロセスが同じデバイスを開こうとすると `Device is busy` で失敗する
(実測で確認済み)。したがって録画は **RTSP 側から** 行う必要がある。

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
  /state/imu /state/thruster_state_all /state/high_power_circuit_info \
  /state/low_power_circuit_info /state/main_power_enabled \
  /perception_node/detections /cmd/target \
  /cmd/direct/thruster_controller/output_lf \
  /cmd/direct/thruster_controller/output_lb \
  /cmd/direct/thruster_controller/output_rb \
  /cmd/direct/thruster_controller/output_rf \
  /joint_states /tf /tf_static
```

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

## rosbag の metadata が書かれないことがある

実機では `ros2 bag record` を SIGINT で止めても **`metadata.yaml` が書かれない**ことがある
(単体で直接 SIGINT を送っても再現。終了に 20 秒かかったうえで `.mcap` だけが残る)。
そのままだと `ros2 bag info` / `ros2 bag play` が
`Could not find metadata in bag directory` で開けない。

**データは失われていない。** MCAP は自己記述形式なので、reindex すれば完全に復元できる:

```bash
ros2 bag reindex <bag ディレクトリ>
```

復元例 (実機、149 秒ぶん): `/state/imu` 6576 件、`/perception_node/detections` 993 件、
`/state/thruster_state_all` 6581 件 — すべて読めるようになる。

`tools/record_run.sh` は停止時にも reindex を試みるが、**シグナルの届き方によっては
停止処理まで到達しないことがある** (バックグラウンド起動時に再現)。確実なのは、
走行後にまとめて直すこと:

```bash
./tools/record_run.sh --fix     # ~/runs 配下で metadata が欠けている bag を全部 reindex
```

**走行のたびに最後に `--fix` を打つ運用にしておけば取りこぼさない。**

## 走行 1 回ぶんをまとめて記録する

```bash
./tools/record_run.sh --name pool-01
```

映像 (前後カメラ、H264 そのまま) と rosbag (状態・指令・検出の 15 トピック) を同時に開始し、
Ctrl-C で両方をきれいに閉じる。出力は `~/runs/<日時>-<名前>/` に
`video/` `bag/` `meta.txt` が揃う。**実機実測で perception への影響なし** (7.74 -> 7.80 Hz)。

## 最低限これだけは残す (競技当日)

1. **`tools/record_run.sh --name <走行名>`** — 映像と bag を同時に開始し、停止時に
   bag の reindex まで済ませる。個別に回すより取りこぼしが少ない
3. `~/umiusi_logs/` のノードログ (`tools/umiusi_stack.sh` が自動で残す)
4. 走行前に `tools/bench_rates.py` を 20 秒回して**その日の周期の記録**を取る
