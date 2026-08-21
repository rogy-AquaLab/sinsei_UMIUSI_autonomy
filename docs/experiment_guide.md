# 実験手順 — rl_attitude と perception を単独で試す

機体を立ち上げて、**姿勢制御と認識をそれぞれ単独で**確認するための手順。
両方を同時に動かすのは、片方ずつ確信が持ててからにする。

前提の環境構築は `robot_setup.md`、性能の目安は `performance_tuning.md`、
記録は `logging.md`。

---

## 0. 立ち上げ

```bash
# [PC] 有線を共有モードに (公式手順の「インターネット共有」)
sudo nmcli con mod "Wired connection 1" ipv4.method shared
sudo ufw allow in on eno1 from 10.42.0.0/24     # ROS 2 の DDS に必要

# [Pi]
ssh pi@alexandrite.local
cd ~/ros2-ws/src/sinsei_UMIUSI_autonomy && ./tools/setup_robot.sh
./tools/acceptance_test.sh
```

共有を切り替えた直後は、Pi が経路変更に適応するまで**数分かかる**。

### まとめて確認したいとき

```bash
./tools/experiment_test.sh            # 事前確認 → perception → 姿勢制御 → ロギング
./tools/experiment_test.sh --perception   # 個別に
```

起動から判定・停止までを自動で通す。**スラスタは回さない**（RL は必ず `--no-publish`）。
以下は同じことを手で、目で見ながらやる手順。

---

## 1. `rl_attitude` を単独で

### 1-1. まず指令を出さずに見る

```bash
./tools/umiusi_stack.sh start --attitude --no-publish   # 計算だけ。カメラも上げない
```

手で組む場合はこちら（内容は同じ）:

```bash
ros2 launch sinsei_umiusi_control main.yaml enable_cameras:=false
ros2 launch umiusi_rl_control rl_attitude.launch.py publish:=false
```

`publish:=false` なので**スラスタには何も出ない**。ログに
`policy loaded from .../export (SB3 非依存の素 torch 推論)` が出れば読めている。

```bash
tail -f /tmp/umiusi_logs/rl.log                                # ポリシー読み込みと棄却率
ros2 topic hz /state/imu                                       # 50 Hz
python3 tools/imu_monitor.py                                   # 傾けて姿勢を目視
```

### 1-2. 目標姿勢の与え方

既定は **水平 (identity) + 前進 0.4 m/s**。実行中に変えるには
`umiusi_rl_control_msgs/AttitudeTarget` を `~/setpoint` に publish する。
クォータニオンを手で組むのは面倒なので、度で指定できるツールを用意した:

```bash
python3 tools/set_attitude.py --level              # 水平・停止
python3 tools/set_attitude.py --yaw 90 --hold      # 右に 90 度 (押し続ける)
python3 tools/set_attitude.py --roll 20 --hold
python3 tools/set_attitude.py --vel 0.3            # 前進速度だけ変える
python3 tools/set_attitude.py --yaw 45 --attitude-only --hold   # 速度は無視させる
```

**`--hold` を使うこと。** QoS の depth が 1 なので、1 発だけだと取りこぼす。
DDS が通っていれば **PC 側からでも打てる**（Pi に入り直さなくてよい）。
`--vel` を付けなければ**速度指令は変更しない**（launch の `vel_cmd` のまま）。
止めたいときは `--level` か `--vel 0`。

launch 時に既定を変えることもできる:

```bash
ros2 launch umiusi_rl_control rl_attitude.launch.py vel_cmd:=0.0 publish:=false
```

### 1-3. 出力を見る

```bash
ros2 topic hz   /cmd/direct/thruster_controller/output_lf       # 50 Hz 目標
ros2 topic echo /cmd/direct/thruster_controller/output_lf       # duty_cycle / angle
```

**機体を傾けたとき、戻す向きに duty が出るか**を見る。シミュレータでは
10〜90 度 (ヨーは 179 度) から復元し、定常誤差 2〜3 度・発散なしを確認済み。
**プールが狭いならヨーが一番振りやすい。**

### 1-4. 実際に回す

モータを繋いで publish を有効にして起動する。**必ず e-stop を手元に**:

```bash
./tools/umiusi_stack.sh stop
./tools/umiusi_stack.sh start --attitude
# 手で組む場合: ros2 launch umiusi_rl_control rl_attitude.launch.py
```

```bash
# 緊急停止（別端末に先に用意しておく）
ros2 topic pub --once /rl_attitude_node/estop std_msgs/msg/Bool "{data: true}"

# 復帰（止めたあと再開するとき）
ros2 topic pub --once /rl_attitude_node/estop std_msgs/msg/Bool "{data: false}"

ros2 service call /rl_attitude_node/arm std_srvs/srv/SetBool "{data: false}"   # 武装解除
ros2 service call /rl_attitude_node/arm std_srvs/srv/SetBool "{data: true}"    # 再武装
```

どちらも同じ武装フラグを操作するだけで**インターロックは無い**（`estop true` のあとでも
`arm true` を呼べば武装する）。混乱を避けるため、**止めた経路と同じ経路で戻す**こと。

`estop` は latched (transient_local) だが、`ros2 topic pub --once` は送信後に終了するため
**latch は残らない**。その状態でノードを再起動すると ARMED で立ち上がる（`start_armed:=false`
で起動すれば DISARMED から始められる）。

---

## 2. `perception` を単独で

```bash
./tools/umiusi_stack.sh stop
./tools/umiusi_stack.sh start --perception   # カメラブリッジ + perception だけ
```

手で組む場合はこちら:

```bash
ros2 launch sinsei_umiusi_control main.yaml enable_cameras:=true \
    cameras_param_file:=$(ros2 pkg prefix umiusi_autonomy)/share/umiusi_autonomy/config/cameras_deploy.yaml
ros2 launch umiusi_autonomy core_autonomy.launch.py use_core:=false use_rosbridge:=false
```

`core_autonomy` はカメラブリッジと perception も起動する。認識だけ見たいので
`use_core:=false`（BT を起動しない）と `use_rosbridge:=false`（UI を起動しない）で CPU を空ける。

> **`cameras_param_file` を渡さないとカメラが開かない。** 実機既定の `params/cameras.yaml` は
> `usb_camera` が `/dev/video2`（unicam = H264 非対応）を指しており、pipeline が開けず RTSP に
> 映像が来ない（`known_issues.md` の B-1）。その状態だと `camera_bridge_node` が
> `ハードウェア経路 ... software に落とします` / `接続できません` を出し続ける。
> `umiusi_stack.sh` は同梱の `cameras_deploy.yaml`（`/dev/video4`）を自動で渡す。
> デバイス番号は挿し順で変わるので `v4l2-ctl --device=/dev/video4 --list-formats` で確認すること。

```bash
ros2 topic hz /front_cam/image_raw            # ブリッジが画像を流しているか
ros2 topic hz /perception_node/detections     # 認識周期
ros2 topic echo --once /perception_node/detections
```

`/cmd/target` が出ないのは**正常** (core の BT が AUTO に入るまで
`auto_target_generator` は activate されない。`--perception` では BT 自体を起動しない)。
単体で見るなら手動で遷移させる (README「`/cmd/target` が出ないとき」)。

### 検出器の切り替え

既定は同梱の `camp_mix.pt`。**実際の水中は `camp_real.pt` のほうが強い** (F1 0.80 vs 0.69):

```bash
ros2 launch umiusi_autonomy core_autonomy.launch.py \
    model_path:=$(ros2 pkg prefix umiusi_autonomy)/share/umiusi_autonomy/models/detector/camp_real.pt
```

---

## 3. 検出結果を目で見る (PC 側)

**表示は PC にやらせる。** Pi 側の追加負荷は「既に publish している画像を 1 つ多く
購読される」分だけで、デコードも描画もウィンドウも PC が持つ。

```bash
# [PC] Pi と DDS が通っている状態で
python3 tools/view_detections.py
```

バウンディングボックスに色・確信度・距離が重なる。`q` で終了。

```bash
python3 tools/view_detections.py --save run.mp4               # 表示しつつ録画
python3 tools/view_detections.py --save run.mp4 --no-window   # 録画だけ (表示しない)
python3 tools/view_detections.py --no-window                  # 統計だけ (ヘッドレス)
```

> **Pi では動かさないこと。** CPU が飽和して認識周期が落ちる。
> 画像と検出は別トピックで、その時点で**最後に届いた検出**を重ねる（時刻照合はしない）。
> 検出は画像より遅いので、同じ枠が数フレーム続くのは正常。

UI (WebRTC) 側でも映像は見えるが、そちらは MediaMTX 経由の生映像で**検出枠は重ならない**。
枠を見たいときはこのツールを使う。

---

## 4. 記録

```bash
./tools/record_run.sh --name <走行名>    # 映像 + bag。Ctrl-C で停止
./tools/record_run.sh --fix              # 走行後。bag の metadata を復元する
```

**`--fix` は毎回打つ。** rosbag2 が metadata を書かないことがあり、そのままだと
`ros2 bag info` で開けない (データ自体は無事)。

---

## 5. 実測値 (2026-08-21、`tools/experiment_test.sh` で確認)

| 項目 | 見かた | 実測 |
|---|---|---|
| **IMU の化けサンプル** | ノードのログに `IMU の異常サンプルを検出` | 手で 150 秒振って **0.44%** (ノルム異常 24 / 角速度スパイク 9)。いずれも読み出し化けで、速い運動ではない。**既定では検出のみで破棄しない** (`imu_sanity_enforce:=true` で破棄。`known_issues.md` A-1) |
| **IMU の姿勢基準の飛び** | `IMU の姿勢基準が飛んだので再同期` | 150 秒に 1 回、169°。**飛んだら目標姿勢を与え直すこと** (飛ぶ前の基準で与えているため) |
| **認識周期が 10 Hz に張り付くか** | `ros2 topic hz /perception_node/detections` | **単体 10.01 Hz で張り付く**（以前は 7.9 Hz）。BT を載せた本番構成では 9.26 Hz / CPU 74.8% |
| **Ctrl-C で録画が閉じるか** | 停止後に `ls ~/runs/*/video/` と `pgrep gst-launch` | **閉じる**。bag の `metadata.yaml` も書かれ reindex 不要、孤児プロセスも残らない |
| **RL の実機での復元** | 傾けて `/cmd/direct/...` の duty | **未確認** — `publish:=false` でしか回していない。手順 1-4 で確認すること |

> **記録は 30 秒以上録ること。** `record_run.sh` は起動に 10 秒以上かかり (`ros2 topic list` と
> カメラの立ち上げ)、`ros2 bag record` の discovery にも数秒かかる。15 秒だと **bag に `/tf` しか
> 入らない**（実機で踏んだ）。30 秒あれば `/state/imu` が 50 Hz、`detections` が 10 Hz で入る。

その他の実測: `/front_cam/image_raw` 15.1 Hz / `/state/imu` 50.2 Hz /
`/state/thruster_state_all` 50.0 Hz / VESC 4 台すべて応答 / CPU 温度 42〜48°C (throttle なし)。
数値の基準は `performance_tuning.md`、確認項目の全体像は `competition_checklist.md`。
