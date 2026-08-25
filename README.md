# sinsei_UMIUSI_autonomy

Experimental UMIUSI autonomy + low-level control, as a **multi-package colcon repo** (one repo, split
packages so the control layer builds independently of perception).

| package | build type | role | key deps |
|---|---|---|---|
| [`umiusi_autonomy`](umiusi_autonomy/) | ament_python | perception + planner: `perception_node`, `camera_bridge_node`, `navigator_node`, on-core `auto_target_generator` | `umiusi_perception` wheel (pip), `umiusi_rl_control` (arm helper) |
| [`umiusi_autonomy_msgs`](umiusi_autonomy_msgs/) | ament_cmake | `BalloonDetection` / `BalloonDetectionArray` (perception -> planner) |
| [`umiusi_rl_control`](umiusi_rl_control/) | ament_python | low-level control: RL attitude controller, keyboard teleop, arm/e-stop | `torch`/`numpy` (pip) — **no perception, no SB3** |
| [`umiusi_rl_control_msgs`](umiusi_rl_control_msgs/) | ament_cmake | `AttitudeTarget` setpoint (attitude quaternion + feed-forward velocity + mask) | geometry_msgs |

Dependency direction: `umiusi_autonomy` (planner) → `umiusi_rl_control` (controllers) → `umiusi_rl_control_msgs`.
The control layer never depends on perception/autonomy, so:

```bash
# build ONLY the low-level control (RL attitude controller etc.) — no perception/sim pulled in:
colcon build --packages-up-to umiusi_rl_control
# full autonomy stack:
colcon build --packages-up-to umiusi_autonomy
```

## 実機で動かす — 導入から単体実験・記録まで

環境構築済みの Pi（公式手順を終え、ROS 2 Jazzy / `can0` / MediaMTX が動いている状態）に
autonomy を載せて、**姿勢制御と認識をそれぞれ単体で**確かめ、記録まで通す流れ。

| 段階 | やること | 詳細 |
|---|---|---|
| 1. 導入 | `cd ~/ros2-ws/src && git clone https://github.com/rogy-AquaLab/sinsei_UMIUSI_autonomy.git`<br>`cd sinsei_UMIUSI_autonomy && ./tools/setup_robot.sh` | [`docs/robot_setup.md`](docs/robot_setup.md) |
| 2. 確認 | `./tools/acceptance_test.sh` — CAN / VESC / カメラ / torch / 周期 / IMU を自動判定 | 〃 |
| 3. 単体実験 | `./tools/experiment_test.sh` で一括確認、または `./tools/umiusi_stack.sh start --attitude` / `--perception`（下記） | [`docs/experiment_guide.md`](docs/experiment_guide.md) |
| 4. 記録 | `./tools/record_run.sh --name <走行名>` → 走行後に `--fix` | [`docs/logging.md`](docs/logging.md) |
| 5. 通し | `./tools/umiusi_stack.sh start`（本番構成） | 下記「launch ファイルの使い分け」 |

検出器も RL ポリシーもリポジトリに同梱しています。
ログは既定で `/tmp/umiusi_logs/{control,core,rl}.log`（`UMIUSI_LOGDIR` で変更可）。

`experiment_test.sh` は起動から判定・停止まで自動で通し、**スラスタは回しません**。
実際に回す確認は [`docs/experiment_guide.md`](docs/experiment_guide.md) の 1-4 を、
e-stop を用意して手順どおりに。

### 姿勢制御 (`rl_attitude`) の単体実験

```bash
./tools/umiusi_stack.sh start --attitude               # control + RL
./tools/umiusi_stack.sh start --attitude --no-publish  # 計算だけ (ドライ試験)
./tools/umiusi_stack.sh stop
```

**起動しただけではスラスタに何も出ません** — `start_armed` の既定が false、`vel_cmd` の既定が
0（姿勢保持のみ）だからです。動かすには武装します:

```bash
ros2 service call /rl_attitude_node/arm std_srvs/srv/SetBool "{data: true}"
python3 tools/set_attitude.py --vel 0.4 --hold    # 前進もさせるなら
```

実行中に効く調整（`ros2 param set`。launch 引数でも指定できます）:

| パラメータ | 既定 | 用途 |
|---|---|---|
| `hold_yaw` | `true` | `false` で **yaw を保持しない**（roll/pitch のみ）。手で回したとき戻そうとして回り続けるのを避ける |
| `max_duty` | `0.25` | `duty_cycle` の絶対値上限（`1.0` = 制限なし）。**0.25 で開始**。0.2 は 96% 飽和で比例制御にならず降下もできない（8/25 の水中 run）。**0.4 は零空間を潰してから** — 上限は力の次元で効くので 0.2→0.4 は 4 倍（issue #19）|
| `servo_slew_deg_per_s` | `250.0` | **サーボ指令のレート制限。sim と同じ値**。0 以下で無効（`known_issues.md` A-11）|
| `thrust_slew_per_s` | `4.0` | ESC 指令のレート制限。同上 |
| `vel_cmd` | `0.0` | 前進速度 [m/s]。新ポリシーは停止保持（0）も学習分布内。巡航試験で 0.4 に上げる |
| `vel_timeout` | `0.0` | 速度指令がこの秒数更新されなければ 0 に戻すデッドマン（0 以下で無効）。狭いプールの巡航試験では `5.0` 推奨 |
| `target_depth` | `0.0` | 深度モード時の目標深度 [m, 正=深い]（`depth_supervisor:=true` 時のみ有効）|

```bash
ros2 param set /rl_attitude_node hold_yaw false
ros2 param set /rl_attitude_node vel_cmd 0.4
```

同梱ポリシーは 4 種（すべて REP-103 観測 + golden.npz 付き。読み込み時に自動検証）:

| バンドル | 観測 | 用途 |
|---|---|---|
| `av_cal1_best_rep103`（既定） | 17 次元（姿勢+速度指令） | 本命 |
| `att_cal1_best_rep103` | 14 次元（姿勢のみ） | フォールバック。巡航が死んでも姿勢試験が成立する |
| `av_sim2real2_rep103` | 17 次元 | B 案（較正前物理で学習、指令が最も滑らか）。A/B 材料 |
| `av_cal5_3d_rep103` | 17 次元（3-D ベクタリング） | EXPERIMENTAL。深度モードの降下バースト専用（`vertical_ok`。上昇・斜めには使わない）|

```bash
./tools/umiusi_stack.sh start --attitude --attitude-policy   # 姿勢保持専用に差し替え
UMIUSI_RL_MODEL=$(ros2 pkg prefix umiusi_rl_control)/share/umiusi_rl_control/models/av_sim2real2_rep103 \
  ./tools/umiusi_stack.sh start --attitude                   # B 案で A/B 試験
```

カメラは上げないので CPU が空きます。**見るところ**:

| 見るもの | コマンド | 期待 |
|---|---|---|
| ポリシーが読めたか | `tail -f /tmp/umiusi_logs/rl.log` | `policy loaded from .../export` |
| IMU が来ているか | `ros2 topic hz /state/imu` | 50 Hz |
| 姿勢が正しいか | `python3 tools/imu_monitor.py` | 傾けた向きと表示が一致 |
| 目標姿勢を与える | `python3 tools/set_attitude.py --yaw 90 --hold` | `--hold` 必須（QoS depth=1）。Pi でも PC でも動く |
| **いまの目標値** | `ros2 topic echo --once /rl_attitude_node/current_setpoint` | latch しているのでいつ繋いでも読める。ログにも度で出る |
| 出力が出ているか | `ros2 topic hz /cmd/direct/thruster_controller/output_lf` | 50 Hz |
| 復元するか | `ros2 topic echo /cmd/direct/thruster_controller/output_lf` | **傾けたら戻す向きに duty**。発散しない |
| IMU の棄却率 | `rl.log` の `IMU サンプルを破棄` | 静止時はほぼ 0（多すぎるなら閾値を緩める）|

スラスタへ出すときは **e-stop を別端末に用意してから**。`teleop_keyboard` を開いておくのが
確実です（正しい QoS で e-stop を打てる）:

```bash
ros2 run umiusi_rl_control teleop_keyboard
# 手で打つなら --qos-durability transient_local が必須 (既定の VOLATILE では届かない)
ros2 topic pub --once --qos-durability transient_local \
    /rl_attitude_node/estop std_msgs/msg/Bool "{data: true}"
```

### `perception` の単体実験

```bash
./tools/umiusi_stack.sh start --perception          # control + カメラブリッジ + perception のみ
./tools/umiusi_stack.sh stop
```

core の BT も UI も起動しないので、認識だけに CPU を使えます。**見るところ**:

| 見るもの | コマンド | 期待 |
|---|---|---|
| RTSP に映像が出ているか | `systemctl is-active mediamtx`<br>`ffprobe -rtsp_transport tcp rtsp://localhost:8554/cam1` | 映像が読める |
| 画像が流れているか | `ros2 topic hz /front_cam/image_raw` | 14 Hz 前後 |
| 認識周期 | `ros2 topic hz /perception_node/detections` | 10 Hz 付近 |
| 中身 | `ros2 topic echo --once /perception_node/detections` | colour / range_m / bbox |
| 目で見る | **[PC 側で]** `python3 tools/view_detections.py` | 枠と確信度が重なる |
| 負荷 | `./tools/umiusi_stack.sh status` | CPU 温度・throttle |

`view_detections.py` は **PC で動かす**こと（Pi で動かすと CPU が飽和して認識周期が落ちる）。
`/cmd/target` が出ないのは正常です（BT を起動していないので `auto_target_generator` は
unconfigured のまま）。

> **カメラが開けず `software に落とします` が出るとき** — 実機既定の `params/cameras.yaml` は
> `usb_camera` が `/dev/video2`（H264 非対応）を指しています（[known_issues B-1](docs/known_issues.md)）。
> `umiusi_stack.sh` は同梱の `umiusi_autonomy/config/cameras_deploy.yaml`（`/dev/video4`）を
> 自動で渡しますが、**手で `ros2 launch` する場合は自分で渡す必要があります**:
>
> ```bash
> ros2 launch sinsei_umiusi_control main.yaml enable_cameras:=true \
>     cameras_param_file:=$(ros2 pkg prefix umiusi_autonomy)/share/umiusi_autonomy/config/cameras_deploy.yaml
> ```
>
> デバイス番号は USB の挿し順で変わります。`v4l2-ctl --device=/dev/video4 --list-formats` で
> H264 が出ることを確認してください。

## launch ファイルの使い分け

ワークスペース全体で launch が 6 本あり、**どれを組み合わせるかで挙動が変わる**。

### このリポジトリの launch

| launch | 起動するもの | 用途 |
|---|---|---|
| **`core_autonomy.launch.py`** | カメラブリッジ + perception + `auto_target_generator` + core の BT / manual_target / low_power + rosbridge | **本番はこれ**。core の BT に載り、AUTO モードで自律が動く |
| `autonomy.launch.py` | カメラブリッジ + perception + `navigator_node` | **core を使わない直接経路**。navigator が `/cmd/direct` でスラスタを直接叩く。単体試験向け |
| `rl_attitude.launch.py` | `rl_attitude_node` | RL 姿勢制御だけ。上の 2 つとは独立に足せる |

主な引数:

| launch | 引数 |
|---|---|
| `core_autonomy` | `model_path` `image_topic` `use_rosbridge` `use_camera_bridge` `use_core` `rtsp_url` |
| `autonomy` | `model_path` `image_topic` `publish` |
| `rl_attitude` | `model_path` `vel_cmd` `publish` `start_armed` `hold_yaw` `max_duty` `vel_timeout` `depth_supervisor` `target_depth` |

`model_path` は未指定なら同梱の検出器 (`models/detector/camp_real2.pt`) を使う。
`publish` を `false` にすると計算だけしてスラスタに指令を出さない (ドライ試験)。
`use_core` を `false` にすると core の BT スタックを起動せず、カメラブリッジ + perception だけになる
(認識の単体実験用。`umiusi_stack.sh start --perception` がこれを使う)。

### 他リポジトリの launch

| launch | 中身 |
|---|---|
| `sinsei_umiusi_control/launch/main.yaml` | **ハードウェア**。CAN / IMU / カメラ / コントローラ。**常に必要** |
| `sinsei_umiusi_core/launch/main.yaml` | core の BT スタック (**`core_autonomy` と併用不可**、下記) |
| `umiusi_sim_bridge/launch/sim.launch.py` | シミュレータ |

### 組み合わせ

```bash
# 本番 (実機)
ros2 launch sinsei_umiusi_control main.yaml enable_cameras:=true \
    cameras_param_file:=$(ros2 pkg prefix umiusi_autonomy)/share/umiusi_autonomy/config/cameras_deploy.yaml
ros2 launch umiusi_autonomy core_autonomy.launch.py
# 姿勢制御も使うなら
ros2 launch umiusi_rl_control rl_attitude.launch.py publish:=false
```

`cameras_param_file` を渡さないと実機既定の `/dev/video2` (H264 非対応) が使われ、
カメラが開けない (known_issues B-1)。`tools/umiusi_stack.sh` は同梱の設定を自動で渡す。

`tools/umiusi_stack.sh` が上の 1〜2 本目 (と `--with-rl` で 3 本目) をまとめて起動する。
単体実験は `--attitude` / `--perception` (上記「実機で動かす」)。

**`sinsei_umiusi_core/launch/main.yaml` と `core_autonomy.launch.py` を同時に起動してはいけない。**
`core_autonomy` は core の `main.yaml` をノード単位で写した上で、AUTO モードの目標生成だけを
core の置き換え用ノードから umiusi_autonomy の FSM 版に差し替えたもの。両方起動すると
**`auto_target_generator` が 2 つ立ち上がり `/cmd/target` を奪い合う**。

`autonomy.launch.py` と `core_autonomy.launch.py` も同時に使わない。前者の `navigator_node` は
`/cmd/direct` へ、後者は core の BT 経由で `/cmd/target` へ指令を出すので、**2 系統が同時に
スラスタを動かすことになる**。

### `/cmd/target` が出ないとき

`core_autonomy` で起動した直後は `/cmd/target` が出ない。**これは正常**で、
`auto_target_generator` は lifecycle ノードであり、**core の BT が AUTO モードに入って初めて
activate される**設計のため。単体で確認したいときは手動で遷移させる:

```bash
ros2 lifecycle set /auto_target_generator configure
ros2 lifecycle set /auto_target_generator activate
```
