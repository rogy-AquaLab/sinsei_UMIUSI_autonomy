# パラメータの置き場所

現場で値を変えたいときに、どのファイルの何を触ればよいか。

| 何を変えたいか | ファイル | ノード | 走らせたまま変わるか |
|---|---|---|---|
| 競技の自律（探索の旋回、接近、突進、後退、青の回避） | `umiusi_autonomy/config/competition.yaml` の `fsm.*` | `/auto_target_generator` | 変わる（`ros2 param set`） |
| AUTO の指令の向き（`surge_sign` / `yaw_rate_scale`）と、検出の途切れの判定 | 同上（`fsm` の外） | 同上 | 変わる |
| 認識（モデル、閾値 `min_confidence`、推論の周期、AUTO 以外で止めるか） | `umiusi_autonomy/config/autonomy.yaml` の `perception_node` | `/perception_node` | 閾値は変わる。モデルは起動し直し |
| カメラブリッジ（fps の間引き、経路 rtsp / shm） | launch 引数 `camera_max_fps` / `camera_source` | `/camera_bridge_node` | 起動し直し |
| 姿勢制御のゲイン、mixer、推力の上限、サーボ | `sinsei_UMIUSI_control/params/controllers.yaml` | `/attitude_controller` ほか | configure 時に読む（再 configure か起動し直し） |
| カメラのパイプライン（解像度、fps、フォーカス） | `sinsei_UMIUSI_control/params/cameras.yaml`（shm 経路は `cameras_shm.yaml`） | `pi_camera` / `usb_camera` | 起動し直し |

## 競技の自律（`competition.yaml`）

- 値はここ 1 箇所。ここに無い調整値は FSM（umiusi_perception の `behavior.py`）の既定値。
- 名前・単位・FSM 側の実体の対応は `umiusi_autonomy/umiusi_autonomy/fsm_params.py`。
- 単位:
  - surge / heave / yaw は FSM の指令 [-1, 1]。surge・heave は Target.velocity の推力の割合、yaw は `yaw_rate_scale` 倍の rad/s
  - `*_deg` は度
  - `*_steps` / `*_frames` は FSM の周期数。目標は 50 Hz だが、CPU が詰まると下がる（10/03 の AUTO は 26 Hz）
- FSM の時間（探索の 1 周の積分など）は、周期の実時間で進む。10/03 までは固定の 0.02 s を足していたため、周期が 26 Hz に落ちると探索の 1 周が実際は 1.9 周になっていた。
- 別の値の組で試すときは、ファイルを複製して `ros2 launch umiusi_autonomy core_autonomy.launch.py competition_params:=/abs/my.yaml`。

```bash
ros2 param list /auto_target_generator | grep fsm.        # 一覧
ros2 param get /auto_target_generator fsm.search_yaw      # 今の値
ros2 param set /auto_target_generator fsm.search_yaw 0.3  # 変える（次の周期から効く）
ros2 param dump /auto_target_generator > my.yaml          # 現場で詰めた値を残す
```
