# umiusi_rl_control — UMIUSI low-level control layer

Self-contained low-level control for the UMIUSI vehicle: a trained **RL attitude(-velocity)
controller**, a **keyboard teleop**, and a shared **arm/disarm (e-stop)**. It **builds independently of
the perception / autonomy stack** — it does NOT depend on `umiusi_perception`; the RL policy needs only
`torch` + `numpy` (pip) at runtime — the bundled policies ship as a plain-torch `export/`
(no SB3/gymnasium on the robot).

```
  setpoint (AttitudeTarget)                 low-level controller            thrusters
  ─ target attitude (quaternion)  ─────►   rl_attitude_node          ─────► /cmd/direct/...
  ─ feed-forward velocity (body)           (RL policy, this package)        (sinsei_umiusi_control)
```

The planners (teleop here, or `umiusi_autonomy`'s navigator) only publish a **setpoint**; this layer
turns it into thruster commands. See the workspace README / project design notes for the target
layering (velocity/cmd_vel → allocator vs. AttitudeTarget → attitude-hold controller).

## Packages
| package | build type | contents |
|---|---|---|
| `umiusi_rl_control_msgs` | ament_cmake (rosidl) | `AttitudeTarget` (target attitude quaternion + feed-forward velocity + `type_mask`) |
| `umiusi_rl_control` | ament_python | `rl_attitude_node`, `teleop_keyboard`, shared `arm` (e-stop), launch, bundled policy |

## The setpoint: `umiusi_rl_control_msgs/AttitudeTarget`
An absolute attitude target + a feed-forward body velocity (modeled on `mavros_msgs/AttitudeTarget`):
`header`, `orientation` (Quaternion, identity = upright), `velocity` (Vector3, target-body frame,
m/s), and `type_mask` (`IGNORE_ATTITUDE=1`, `IGNORE_VELOCITY=2`; a masked-out field keeps its previous
value, default `0` = update both).

## Build (independent)
```bash
cd ros2_ws
colcon build --packages-select umiusi_rl_control_msgs umiusi_rl_control
source install/setup.bash
# RL runtime deps (pip, not rosdep keys):
pip install torch numpy
```

## Run
```bash
# RL attitude controller with the bundled av_cal1_best_rep103 policy (holds upright; vel_cmd 0):
ros2 launch umiusi_rl_control rl_attitude.launch.py
ros2 launch umiusi_rl_control rl_attitude.launch.py vel_cmd:=0.4    # cruise +X
ros2 launch umiusi_rl_control rl_attitude.launch.py publish:=false  # predict only (no thrusters)
ros2 launch umiusi_rl_control rl_attitude.launch.py vel_timeout:=5.0 # デッドマン: 速度指令が
                                    # 5 s 更新されなければ自動で 0 に戻す (狭いプールの巡航試験で推奨)
```
Requires the controllers/bridge (`sinsei_umiusi_control` or `umiusi_sim_bridge`) providing
`/state/imu` and consuming `/cmd/direct/...`.

**Policy bundles** (`models/`): `av_cal1_best_rep103` (default, 17-D attitude+velocity),
`att_cal1_best_rep103` (14-D attitude-only fallback), `av_sim2real2_rep103` (17-D, plan B),
`av_cal5_3d_rep103` (17-D 3-D vectoring, **EXPERIMENTAL / 深度モードの降下バースト専用**),
`av_mode13` (18-D 観測 + **6 次元のレンチモードレート** `action_mode: modes`)。

`av_mode13` は 8/25 実機で鉛直パワーの 41% を占めた零空間を、**それを表現できない action 基底へ
張り替えて**構造的に潰したもの (sim 実測 5.7%)。出力が [servo x4, esc x4] ではないので、
ノードが `meta.json` の `action_contract` に従って積分 → ミキサ → 折返しを再現する
(`mode_action.py`)。**既定はまだ `av_cal1_best_rep103`** — 実機未検証のため、使うときは明示する:

```bash
UMIUSI_RL_MODEL=$(ros2 pkg prefix umiusi_rl_control)/share/umiusi_rl_control/models/av_mode13 \
  ./tools/umiusi_stack.sh start --attitude
```
All consume **REP-103 body-frame** observations (`export/meta.json` `obs_frame: rep103` is
enforced at load) and carry `golden.npz` sim-recorded obs→action vectors that the node replays
at load — a mismatch refuses to run (deploy-time verification, issue #15 A-5).

**観測レイアウト** — 次元でタスクを判別し、組み立てを合わせる:

| 次元 | 並び | タスク |
|---|---|---|
| 18 | `ori_err(3) gyro(3) v_cmd(3) prev_action(8) max_duty(1)` | attitude_velocity + duty 上限 |
| 17 | `ori_err(3) gyro(3) v_cmd(3) prev_action(8)` | attitude_velocity (巡航) |
| 14 | `ori_err(3) gyro(3) prev_action(8)` | attitude (姿勢のみ) |

**`max_duty` は必ず末尾** (prev_action の後ろ)。sim 側の warm start が 17 次元の学習済み重みを
初層のゼロパディングで引き継ぐため、この位置は**不変の契約**。値は正規化なしで、
実行中に `ros2 param set /rl_attitude_node max_duty 0.3` で変えると観測にも反映される
— 「現場で上限を上げたら実際に速く動く」ようにするのが 18 次元化の目的 (盲目の domain
randomization だと方策は最低上限に張り付く)。

**ただし観測に入る値は学習分布 `[0.2, 0.4]` にクランプする。** duty のクリップ自体はオペレータが
設定した `max_duty` のままで、**観測に入れる値だけ**を丸める。範囲外をそのまま入れると、warm
start でゼロパディングされた新次元へ学習時に一度も見ていない値が入り、出力全体が予測不能に
なるため (17 次元時代は単にクリップが緩むだけの単調な変化だった)。範囲外を設定すると警告が出る。

`obs_fields` は **18 次元では必須**。既存の 17/14 は警告のみで通す (後方互換で許してよいのは
「既に出回っていて直せない」バンドルの話で、18 次元はこの仕組みと同時に生まれた)。

水平ポリシーと vert ポリシーで次元が違っていてよい (17 と 18 の混在)。観測はモデルごとに
`model.obs_dim` で組むので `prev_action` の位置はずれない。**ただし vert が 17 次元なら
duty 上限の変化には追従しない** (観測に持たないため) — 深度モードで `max_duty` を上げても
降下バーストだけは挙動が変わらない。

**`golden.npz` は観測の組み立てを検証しない。** 記録済みの観測ベクトルをそのままネットに流す
ので、重みと正規化統計は検証できるが、**このノードが観測をどの順で組むかは見ていない** —
並びを取り違えても golden は PASS する。そこを埋めるのが `export/meta.json` の `obs_fields`
(`[["ori_err",3],["gyro",3],...]`) で、読み込み時に `OBS_FIELDS` と厳密に照合し、食い違えば
起動しない。`obs_fields` が無いバンドルは警告だけ出して通す (既存 17/14 との後方互換)。

### 深度モード切替 (水圧センサ搭載時のみ)
```bash
# 水圧の外側ループで水平巡航 (av_cal1_best) と降下バースト (av_cal5_3d) を切り替える。
# **max_duty 0.4 が前提** — sim 実測では 0.2 だと下向き推力が浮力に負けて降下できない
ros2 launch umiusi_rl_control rl_attitude.launch.py depth_supervisor:=true max_duty:=0.4
ros2 param set /rl_attitude_node target_depth 1.0     # 潜行 (m, 正=深い)
ros2 service call /rl_attitude_node/zero_depth std_srvs/srv/Trigger   # 水面でゼロ点取り直し
ros2 topic echo /rl_attitude_node/depth_mode          # horiz / brake / vert / ascend
```
降下 = ブレーキ 1 s → 3-D ポリシーの純下バースト、浮上 = ホールドして**弱正浮力に任せる**
(受動)。斜め指令は作らない。状態機械・検証済みパラメータ・前提 (機体を弱正浮力にトリム
しておく) は `umiusi_rl_control/depth_supervisor.py` 冒頭と issue #15 のコメント
(sim リハーサル結果) を参照。深度ゼロ点は起動後の最初の水圧サンプルで自動キャプチャ
(水面で起動する前提)。

**Real-time setpoint** (last message wins; defaults = upright + `vel_cmd`):
```bash
ros2 topic pub /rl_attitude_node/setpoint umiusi_rl_control_msgs/msg/AttitudeTarget \
  "{orientation: {x: 0, y: 0, z: 0, w: 1}, velocity: {x: 0.3, y: 0, z: 0}, type_mask: 0}"
```

### Keyboard teleop + emergency stop
Run in its **own terminal** (needs the keyboard):
```bash
ros2 run umiusi_rl_control teleop_keyboard
```
`w/s a/d r/f` = ±velocity x/y/z (body), `i/k j/l u/o` = pitch/yaw/roll target, `SPACE` = zero velocity,
`t` = upright, **`x` = EMERGENCY STOP**, `z` = re-arm, `q` = quit. It publishes an `AttitudeTarget`
setpoint; the e-stop both signals the controller to disarm **and** directly detaches the thrusters.

### Arm / disarm (e-stop)
`rl_attitude_node` (and `umiusi_autonomy`'s `navigator_node`, which reuses this package's helper)
exposes a latched safety interface: publish `~/estop` (`std_msgs/Bool` `true`) or call `~/arm`
(`std_srvs/SetBool` `data: false`) to DISARM — the node stops and asserts a **detach** every tick
(`ThrusterOutput` `runnable.esc=servo=false` + zero, so the control stack releases esc/servo). Re-arm
with `~/arm` `data: true` (or `~/estop` `false`). `start_armed:=false` launches disarmed. (Core's
power-off / Standby is the other, stack-wide stop.)

## Unit / frame caveats
- IMU quaternion + `angular_velocity` go into the observation **unconverted** — the frame contract
  is "the IMU publishes REP-103 (x fwd / y left / z up), rad/s". Verify the mount before a test
  (issue #15 A-4); if it is off, fix the IMU driver (AXIS_MAP), not this node.
- servo output `ThrusterOutput.angle` is published in DEGREES (= action × `servo_range_deg`); msg documents rad.

## License
MIT.
