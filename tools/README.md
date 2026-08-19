# tools

実機の立ち上げ・動作確認・記録に使うスクリプト。ROS 2 環境を source してから実行する。

## 起動と確認

| スクリプト | 用途 |
|---|---|
| `umiusi_stack.sh` | **実機スタックの起動/停止/状態確認**。`start` / `stop` / `restart` / `status`、`--no-ui` で rosbridge を止めて CPU を空ける、`--with-rl` で RL 姿勢制御も起動。`timeout N ros2 launch &` で起動すると計測の途中で寿命が切れて結果が壊れるので、起動はここに寄せること |
| `acceptance_test.sh` | **受け入れ試験**。CAN / VESC 4 台の ping / カメラ / torch / 周期 / IMU 健全性 を一気に確認して OK/NG を出す。`--start` でスタック起動から行う |
| `bench_rates.py` | 指定トピックの周期と CPU/温度を確実に測る。**publisher 数も報告する**ので「0 Hz なのは publisher が居ないからか、遅いだけか」を取り違えない |

## 記録

| スクリプト | 用途 |
|---|---|
| `record_camera.sh` | **カメラ映像の録画**。実機カメラは ROS トピックを出さず rosbag に残らないため、RTSP から H264 のまま録る (再エンコードなし、CPU 15.5%)。`--raw` は切り捨てに強い生 H264 で、`kill -9` や電源断でも壊れない。詳細は `docs/logging.md` |

## センサ確認

| スクリプト | 用途 |
|---|---|
| `imu_monitor.py` | `/state/imu` の roll/pitch/yaw をライブ表示。機体を傾けながら目視確認する用 |
| `imu_trace.py` | 傾け→復帰の軌跡を 50 Hz 全数記録し、復帰誤差を数値で出す。動きを検知したら自動で記録開始 |
| `imu_glitch.py` | 静止状態での IMU データ健全性 (ジャイロ異常値・クォータニオンのノルム異常) を計測 |

## 開発・試験用

| スクリプト | 用途 |
|---|---|
| `sim_image_pub.py` | PC から画像を `/front_cam/image_raw` へ配信する。実機カメラが無い場所で perception を試すとき用。**QoS は RELIABLE 既定** (perception_node の購読に一致させないと 1 枚も届かない) |
| `policy_infer.py` | SB3 非依存の PPO(MlpPolicy) 推論。実機の numpy 1.26 では SB3 の policy zip が読めないため、`umiusi_sim/tools/export_policy.py` が書き出した `weights.pt` + `obs_norm.npz` を torch だけで読む |

## 典型的な使い方

```bash
# 1. 起動 (UI を使わないなら --no-ui で CPU が空く)
./umiusi_stack.sh start --no-ui

# 2. 受け入れ試験
./acceptance_test.sh

# 3. 記録しながら走行
./record_camera.sh --raw &
ros2 bag record -o run_$(date +%Y%m%d-%H%M%S) /state/imu /perception_node/detections /cmd/target

# 4. 停止
./umiusi_stack.sh stop
```

関連ドキュメント: `docs/robot_setup.md` (セットアップ手順) /
`docs/performance_tuning.md` (性能チューニング) / `docs/logging.md` (記録) /
`docs/competition_checklist.md` (競技前の確認項目) / `docs/known_issues.md` (既知の問題)
