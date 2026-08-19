# tools

実機 bring-up・動作確認に使うスクリプト。ROS 2 環境を source してから実行する。

| スクリプト | 用途 |
|---|---|
| `sim_image_pub.py` | PC から画像を `/front_cam/image_raw` へ配信する。実機カメラは RTSP にしか流れず ROS トピックが無いため、perception の試験にはこれを使う。**QoS は RELIABLE 既定**(perception_node の購読に一致) |
| `imu_monitor.py` | `/state/imu` の roll/pitch/yaw をライブ表示。機体を傾けながら目視確認する用 |
| `imu_trace.py` | 傾け→復帰の軌跡を 50 Hz 全数記録し、復帰誤差を数値で出す。動きを検知したら自動で記録開始 |
| `imu_glitch.py` | 静止状態での IMU データ健全性 (ジャイロ異常値・クォータニオンのノルム異常) を計測 |
| `policy_infer.py` | SB3 非依存の PPO(MlpPolicy) 推論。実機の numpy 1.26 で SB3 の policy zip が読めない問題の回避に使う。`export_policy.py` (umiusi_sim) が書き出した `weights.pt` + `obs_norm.npz` を読む |

使用例と背景は `docs/robot_setup.md` / `docs/bringup_experiment_2026-08-19.md` を参照。
