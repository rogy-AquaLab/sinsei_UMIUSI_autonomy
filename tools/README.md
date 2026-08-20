# tools

実機の立ち上げ・動作確認・記録に使うスクリプト。ROS 2 環境を source してから実行する。

## セットアップ

| スクリプト | 用途 |
|---|---|
| `setup_robot.sh` | **clone 直後から動く状態まで 1 コマンド**。依存解決 → torch(CPU版) → `umiusi_perception` → ビルド → 確認。**システムのファイルは書き換えず、Python の依存は `--user` (~/.local) に入る**。`--check` で現状確認のみ、`--perception <dir>` でローカルから perception を入れる |

## 起動と確認

| スクリプト | 用途 |
|---|---|
| `umiusi_stack.sh` | **実機スタックの起動/停止/状態確認**。`start` / `stop` / `restart` / `status`、`--no-ui` で rosbridge を止めて CPU を空ける、`--with-rl` で RL 姿勢制御も起動。**単体実験は `--attitude`（姿勢制御だけ・カメラを上げない）/ `--perception`（カメラブリッジ + perception だけ）**、`--publish` で RL の指令を実際に出す。カメラ設定は同梱の `cameras_deploy.yaml` を自動で渡す（`UMIUSI_CAMERAS_PARAM` で上書き）。`timeout N ros2 launch &` で起動すると計測の途中で寿命が切れて結果が壊れるので、起動はここに寄せること |
| `experiment_test.sh` | **単体実験モードの実機確認を一気に通す**。事前確認 (cameras 設定と H264 デバイスの一致など) → perception 単体 → 姿勢制御単体 → ロギング を起動から判定までやる。**スラスタは回さない** (RL は publish=false 固定)。`--perception` / `--attitude` / `--logging` で個別実行 |
| `acceptance_test.sh` | **受け入れ試験**。CAN / VESC 4 台の ping / カメラ / torch / 周期 / IMU 健全性 を一気に確認して OK/NG を出す。`--start` でスタック起動から行う |
| `bench_rates.py` | 指定トピックの周期と CPU/温度を確実に測る。**publisher 数も報告する**ので「0 Hz なのは publisher が居ないからか、遅いだけか」を取り違えない |

## 記録

| スクリプト | 用途 |
|---|---|
| `record_camera.sh` | **カメラ映像の録画**。実機カメラは ROS トピックを出さず rosbag に残らないため、RTSP から H264 のまま録る (再エンコードなし、CPU 15.5%)。`--raw` は切り捨てに強い生 H264 で、`kill -9` や電源断でも壊れない。詳細は `docs/logging.md` |

## 実験用

| スクリプト | 用途 |
|---|---|
| `set_attitude.py` | **rl_attitude の目標姿勢を roll/pitch/yaw [deg] で与える**。素だと `~/setpoint` にクォータニオンを publish する必要があるので、その代わり。`--hold` で押し続ける（QoS depth=1 なので 1 発だと取りこぼす）、`--level` で水平・停止。`--vel` 未指定なら速度指令は変更しない |
| `view_detections.py` | **検出結果を画像に重ねて表示**（時刻照合はせず、最後に届いた検出を重ねる）。**PC 側で動かす**こと（Pi でやると CPU が飽和して認識周期が落ちる）。`--save` で mp4 保存（表示も続く。録画だけなら `--no-window` を併用）|

## センサ確認

| スクリプト | 用途 |
|---|---|
| `imu_monitor.py` | `/state/imu` の roll/pitch/yaw をライブ表示。機体を傾けながら目視確認する用 |
| `imu_trace.py` | 傾け→復帰の軌跡を 50 Hz 全数記録し、復帰誤差を数値で出す。動きを検知したら自動で記録開始 |
| `imu_glitch.py` | 静止状態での IMU データ健全性 (ジャイロ異常値・クォータニオンのノルム異常) を計測 |
| `imu_sanity_check.py` | **機体を動かしながら**サニティフィルタの挙動を見る。`rl_attitude_node` と同じ `ImuSanity` を同じ QoS で通し、閾値までの余裕を倍率で出す。連続棄却と再同期の回数も報告 |
| `imu_sanity_replay.py` | 記録した bag に**フィルタをかけ直す** (PC で動く)。`--sweep` で閾値を振って棄却の出かたを比べられるので、実機で測り直さずに閾値を詰められる |

## 開発・試験用

| スクリプト | 用途 |
|---|---|
| `sim_image_pub.py` | PC から画像を `/front_cam/image_raw` へ配信する。実機カメラが無い場所で perception を試すとき用。**QoS は RELIABLE 既定** (perception_node の購読に一致させないと 1 枚も届かない) |
| `infer_bench.py` | 検出器の 1 フレーム推論時間をスレッド数別に測る。実機ではスレッドを増やすほど遅くなる (`docs/performance_tuning.md`) ことを確認できる |

> SB3 非依存の推論実装は `umiusi_rl_control/policy_infer.py` (パッケージ内) にある。
> `rl_attitude_node` が `export/` を自動で見つけて使うので、通常は意識しなくてよい。

## 典型的な使い方

```bash
# 1. 起動 (UI を使わないなら --no-ui で CPU が空く)
./umiusi_stack.sh start --no-ui

# 2. 受け入れ試験
./acceptance_test.sh

# 2-2. 単体実験をまとめて確認する (起動 → 判定 → 停止まで自動)
./experiment_test.sh

# 2-3. 手で見る (見かたは docs/experiment_guide.md)
./umiusi_stack.sh restart --attitude
./umiusi_stack.sh restart --perception

# 3. 記録しながら走行
./record_camera.sh --raw &
ros2 bag record -o run_$(date +%Y%m%d-%H%M%S) /state/imu /perception_node/detections /cmd/target

# 4. 停止
./umiusi_stack.sh stop
```

関連ドキュメント: `docs/experiment_guide.md` (実験手順) / `docs/robot_setup.md` (セットアップ) /
`docs/performance_tuning.md` (性能チューニング) / `docs/logging.md` (記録) /
`docs/competition_checklist.md` (競技前の確認項目) / `docs/known_issues.md` (既知の問題)
