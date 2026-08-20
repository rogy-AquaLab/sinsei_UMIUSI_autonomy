# sinsei_UMIUSI_autonomy

Experimental UMIUSI autonomy + low-level control, as a **multi-package colcon repo** (one repo, split
packages so the control layer builds independently of perception).

| package | build type | role | key deps |
|---|---|---|---|
| [`umiusi_autonomy`](umiusi_autonomy/) | ament_python | perception + planner: `perception_node`, `camera_bridge_node`, `navigator_node`, on-core `auto_target_generator` | `umiusi_perception` wheel (pip), `umiusi_rl_control` (arm helper) |
| [`umiusi_autonomy_msgs`](umiusi_autonomy_msgs/) | ament_cmake | `BalloonDetection` / `BalloonDetectionArray` (perception -> planner) |
| [`umiusi_rl_control`](umiusi_rl_control/) | ament_python | low-level control: RL attitude controller, keyboard teleop, arm/e-stop | `stable-baselines3`/`torch`/`gymnasium` (pip) — **no perception** |
| [`umiusi_rl_control_msgs`](umiusi_rl_control_msgs/) | ament_cmake | `AttitudeTarget` setpoint (attitude quaternion + feed-forward velocity + mask) | geometry_msgs |

Dependency direction: `umiusi_autonomy` (planner) → `umiusi_rl_control` (controllers) → `umiusi_rl_control_msgs`.
The control layer never depends on perception/autonomy, so:

```bash
# build ONLY the low-level control (RL attitude controller etc.) — no perception/sim pulled in:
colcon build --packages-up-to umiusi_rl_control
# full autonomy stack:
colcon build --packages-up-to umiusi_autonomy
```

`umiusi_rl_control` is intentionally a **separate, experiment-friendly** package that drives
`sinsei_umiusi_control`'s direct-override interface (it does NOT reimplement that stack); it can later
fold into `sinsei_umiusi_control` as an alternative controller, or be pulled in as a dependency.

See each package's README for details. Requires the vendored `sinsei_UMIUSI_*` packages + the
`umiusi_perception` wheel (for `umiusi_autonomy`) — see the workspace `ros2_ws/README.md`.

## launch ファイルの使い分け

ワークスペース全体で launch が 6 本あり、**どれを組み合わせるかで挙動が変わる**。

### このリポジトリの launch

| launch | 起動するもの | 用途 |
|---|---|---|
| **`core_autonomy.launch.py`** | カメラブリッジ + perception + `auto_target_generator` + core の BT / manual_target / low_power + rosbridge | **本番はこれ**。core の BT に載り、AUTO モードで自律が動く |
| `autonomy.launch.py` | perception + `navigator_node` | **core を使わない直接経路**。navigator が `/cmd/direct` でスラスタを直接叩く。単体試験向け |
| `rl_attitude.launch.py` | `rl_attitude_node` | RL 姿勢制御だけ。上の 2 つとは独立に足せる |

主な引数:

| launch | 引数 |
|---|---|
| `core_autonomy` | `model_path` `image_topic` `use_rosbridge` `use_camera_bridge` `rtsp_url` |
| `autonomy` | `model_path` `image_topic` `publish` |
| `rl_attitude` | `model_path` `vel_cmd` `publish` |

`model_path` は未指定なら同梱の検出器 (`models/detector/camp_mix.pt`) を使う。
`publish` を `false` にすると計算だけしてスラスタに指令を出さない (ドライ試験)。

### 他リポジトリの launch

| launch | 中身 |
|---|---|
| `sinsei_umiusi_control/launch/main.yaml` | **ハードウェア**。CAN / IMU / カメラ / コントローラ。**常に必要** |
| `sinsei_umiusi_core/launch/main.yaml` | core の BT スタック (**`core_autonomy` と併用不可**、下記) |
| `umiusi_sim_bridge/launch/sim.launch.py` | シミュレータ |

### 組み合わせ

```bash
# 本番 (実機)
ros2 launch sinsei_umiusi_control main.yaml enable_cameras:=true
ros2 launch umiusi_autonomy core_autonomy.launch.py
# 姿勢制御も使うなら
ros2 launch umiusi_rl_control rl_attitude.launch.py publish:=false
```

`tools/umiusi_stack.sh` が上の 1〜2 本目 (と `--with-rl` で 3 本目) をまとめて起動する。

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
