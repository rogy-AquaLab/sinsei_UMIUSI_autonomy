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

## 0-2. 次回実験

当日の実行チェックリスト (実験 1〜8 の順番と合否条件) は **issue #18**。
フロー計測の準備と露光の話は `logging.md`「対地速度をフローで測る準備」。

この手順書に無い道具:

* `tools/thruster_cmd.py` — 較正用のスラスタ直接指令。
  **`rl_attitude_node` と同時に動かさないこと** (同じトピックを奪い合う)。
* `tools/bag_check.py` — bag のその場検品。撤収前に通す。

持ち帰るもの:

1. `rl.log` (`~/umiusi_logs/`)。起動直後の 2 行でどのポリシーで走ったかが決まる。
2. `record_run.sh --flow` で 1 本 (下カメラの mp4 + 露光の実値)。
3. 走行後に `record_run.sh --fix`、その場で `bag_check.py`。

---

## 1. `rl_attitude` を単独で

### 1-1. まず指令を出さずに見る

```bash
ros2 launch umiusi_autonomy stack.launch.py mode:=attitude publish:=false   # 計算だけ。カメラも上げない
```

手で組む場合はこちら（内容は同じ）:

```bash
ros2 launch sinsei_umiusi_control main.yaml enable_cameras:=false
ros2 launch umiusi_rl_control rl_attitude.launch.py publish:=false
```

`publish:=false` なので**スラスタには何も出ない**。ログに
`golden 検証 PASS` → `policy loaded from .../export (obs 17-D, rep103)` が出れば、
ポリシーが読めて配備前検証 (sim の golden vectors と一致) も通っている。

```bash
tail -f /tmp/umiusi_logs/rl.log                                # ポリシー読み込みと棄却率
ros2 topic hz /state/imu                                       # 50 Hz
python3 tools/imu_monitor.py                                   # 傾けて姿勢を目視
```

### 1-2. 目標姿勢の与え方

既定は水平 (identity) + 前進 0。実行中に変えるには
`umiusi_rl_control_msgs/AttitudeTarget` を `~/setpoint` に publish する。

**普段は `teleop_keyboard` を使う。** 目標入力と e-stop が同じ端末に揃う:

```bash
ros2 run umiusi_rl_control teleop_keyboard
```

**数値を決めて再現したいとき**は `set_attitude.py`。度で指定できる:

```bash
python3 tools/set_attitude.py --level              # 水平・停止
python3 tools/set_attitude.py --yaw 90 --hold      # 右に 90 度
python3 tools/set_attitude.py --vel 0.3            # 前進速度だけ変える
python3 tools/set_attitude.py --vel 0 0 -0.2       # 3 成分指定 (X Y Z) = 純下降
```

`--hold` を付けること。QoS の depth が 1 なので 1 発だと取りこぼす。
DDS が通っていれば PC 側からでも打てる。
`--vel X Y Z` の 3 成分形は 3-D ポリシー `av_cal5_3d_rep103` 限定で、それ以外の
バンドルではノード側の interlock が z を 0 にクランプする。

いま何を目標にしているかは latch されているのでいつでも読める:

```bash
ros2 topic echo --once /rl_attitude_node/current_setpoint
```

### 1-3. 出力を見る

```bash
ros2 topic hz   /cmd/direct/thruster_controller/output_lf       # 50 Hz 目標
ros2 topic echo /cmd/direct/thruster_controller/output_lf       # duty_cycle / angle
```

**機体を傾けたとき、戻す向きに duty が出るか**を見る。シミュレータでの現行バンドルの成績は
`att_cal1_best_rep103` が hold 99.5 % / 定常誤差 3.1° / wobble 0.05 rad/s（姿勢専用）、
`av_cal1_best_rep103` が姿勢+速度指令の本命（巡航時の姿勢維持は `max_duty` 0.2 で緩和される）。
**プールが狭いならヨーが一番振りやすい。**

### 1-4. 実際に回す

モータを繋いで起動する。**既定は disarmed + 前進 0 なので、起動しただけでは何も出ない。**
arm して初めて姿勢保持が始まる。**必ず e-stop を手元に**:

```bash
# Ctrl-C で止めてから publish 付きで上げ直す
ros2 launch umiusi_autonomy stack.launch.py mode:=attitude
# 手で組む場合: ros2 launch umiusi_rl_control rl_attitude.launch.py

# arm して初めて動き出す
ros2 service call /rl_attitude_node/arm std_srvs/srv/SetBool "{data: true}"

# 前進もさせるなら (既定は 0 = 姿勢保持のみ)
python3 tools/set_attitude.py --vel 0.4 --hold
```

```bash
# 緊急停止（別端末に先に用意しておく）
ros2 topic pub --once --qos-durability transient_local \
    /rl_attitude_node/estop std_msgs/msg/Bool "{data: true}"

# 復帰（止めたあと再開するとき）
ros2 topic pub --once --qos-durability transient_local \
    /rl_attitude_node/estop std_msgs/msg/Bool "{data: false}"

ros2 service call /rl_attitude_node/arm std_srvs/srv/SetBool "{data: false}"   # disarm
ros2 service call /rl_attitude_node/arm std_srvs/srv/SetBool "{data: true}"    # 再度 arm
```

> **`--qos-durability transient_local` は必須。** `estop` の購読側は latch するために
> TRANSIENT_LOCAL で待っており、`ros2 topic pub` の既定 (VOLATILE) では **QoS が合わず
> マッチしない**。`Waiting for at least 1 matching subscription(s)...` が出続けて
> **緊急停止が届かない**。`ros2 topic info -v` で購読側の Durability を確認できる。
>
> 手打ちは間違えるので、**回すときは `teleop_keyboard` を開いておくほうが安全**:
> `ros2 run umiusi_rl_control teleop_keyboard`（正しい QoS で e-stop を打てる）。

どちらも同じarm フラグを操作するだけで**インターロックは無い**（`estop true` のあとでも
`arm true` を呼べばarm する）。混乱を避けるため、**止めた経路と同じ経路で戻す**こと。

`estop` は latched (transient_local) だが、`ros2 topic pub --once` は送信後に終了するため
**latch は残らない**。その状態でノードを再起動すると ARMED で立ち上がる（`start_armed:=false`
で起動すれば DISARMED から始められる）。

---

## 2. `perception` を単独で

```bash
ros2 launch umiusi_autonomy stack.launch.py mode:=perception   # カメラブリッジ + perception だけ
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
> `stack.launch.py` は同梱の `cameras_deploy.yaml`（`/dev/video4`）を既定で渡す。
> 別のデバイスなら `cameras_param_file:=<path>`。
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

既定は同梱の `camp_real2.pt` (8/25 プール実写で継続学習した版、conf 0.4 で F1 0.80)。
旧版と A/B するとき (**`conf_thresh` も checkpoint 側で 0.4 → 0.3 に変わる**ので、
モデルと閾値の 2 つが同時に動く):

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

| 項目 | 実測 |
|---|---|
| 認識周期 | 単体 **10.01 Hz** で張り付く。BT を載せた本番構成では 9.26 Hz / CPU 74.8% |
| Ctrl-C で録画が閉じるか | 閉じる。`metadata.yaml` も書かれ、孤児プロセスも残らない |
| RL の実機での復元 | **未確認** — `publish:=false` でしか回していない。手順 1-4 で確認する |
| その他 | `/front_cam/image_raw` 15.1 Hz / `/state/imu` 50.2 Hz / VESC 4 台応答 / CPU 42〜48°C |

IMU の化けサンプルと姿勢基準の飛びは `known_issues.md` A-1。

> **記録は 30 秒以上録ること。** カメラの立ち上げと `ros2 bag record` の discovery に
> 数秒かかる。15 秒だと bag に `/tf` しか入らない (実機で踏んだ)。

`record_run.sh` の起動タイミングと購読の確認については、そのスクリプトの冒頭を読むこと。
数値の基準は `performance_tuning.md`、確認項目の全体像は `competition_checklist.md`。
