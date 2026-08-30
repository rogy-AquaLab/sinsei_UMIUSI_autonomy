# umiusi_autonomy — deploy-side perception + navigation nodes

Two thin rclpy nodes that run the **same** ROS-free perception + autonomy code as the in-sim run
(`umiusi_sim/tools/autonomy_run.py`), on the real robot:

```
 onboard camera            perception_node                 navigator_node                sinsei_umiusi_control
 sensor_msgs/Image  ─────▶ learned detector +      ─────▶  behaviour FSM +        ─────▶ /cmd/direct/thruster_
 (/front_cam/image_raw)    sanitise_near_colours          feedforward_allocation         controller/output_{lf,lb,rb,rf}
                           │                               ▲  sensor_msgs/Imu (yaw rate)  (ThrusterOutput, direct override)
                           └──▶ BalloonDetectionArray ─────┘
```

* **`perception_node`** subscribes the onboard camera, runs the learned detector
  (`umiusi_perception.learned_detector`) + the near red/blue colour re-confirmation
  (`umiusi_perception.sanitise_near_colours`), and publishes `umiusi_autonomy_msgs/BalloonDetectionArray`.
* **`navigator_node`** subscribes the detections + IMU, runs the shared FSM
  (`umiusi_perception.autonomy.BalloonBehavior`), and (default `command_mode: direct`) maps its
  `{surge, heave, yaw}` command through `umiusi_perception.control.feedforward_allocation` to the four
  **direct-override** `ThrusterOutput` topics — the identical drive path `tools/ros_policy.py` uses, so
  it drives the existing sinsei_umiusi_control stack **unchanged**.
  * `command_mode: target` (EXPERIMENTAL) instead publishes a `sinsei_umiusi_msgs/Target` on
    `/cmd/target` and lets the control stack allocate — i.e. autonomy **rides on core** (power/mode
    pipeline) rather than overriding thrusters. Not yet behaviour-equivalent to `direct`: it needs
    core powered-on + in AUTO, the stock `auto_target_generator` replaced, and the control-side
    servo-unit / ESC clamp / thrust-sign reconcile. Validate on sim/hardware before use — see the
    `navigator_node` module docstring.
* **`auto_target_generator`** (EXPERIMENTAL) is the **core-integrated** on-core path: a lifecycle
  node that drop-in replaces `sinsei_umiusi_core`'s placeholder `auto_target_generator` (same node
  name + lifecycle contract), so core's behaviour tree activates it on entering AUTO. It runs the
  FSM and publishes `Target` on `/cmd/target` while active — power/mode/thruster-enable stay in
  core. Bring it up **without modifying core** via `launch/core_autonomy.launch.py`, which starts the
  core strategy stack (health check, manual generator, robot_strategy, rosbridge) with this generator
  in place of core's placeholder + `perception_node` (do NOT also run core's `main.yaml` — a second
  generator would race on `/cmd/target`). Needs the same control-side reconcile as `navigator_node`'s
  `target` mode; validate on sim/hardware first.
The **low-level control layer** (RL attitude controller, keyboard teleop, arm/e-stop) moved to the
separate **`umiusi_rl_control`** package so it builds independently of perception — see its README. This
package (`umiusi_autonomy`) is the perception + planner layer; `navigator_node` reuses
`umiusi_common` の共有 arm/e-stop ヘルパ。

All detection/decision/allocation logic lives in the installable `umiusi_perception` package (detector +
FSM + the numpy-only `umiusi_perception.control` allocation); these nodes only do topic plumbing + message
conversion. That is why the perception and behaviour are bit-identical between simulation and the robot.

## Packages
| package | build type | contents |
|---|---|---|
| `umiusi_autonomy_msgs` | ament_cmake (rosidl) | `BalloonDetection`, `BalloonDetectionArray` |
| `umiusi_autonomy` | ament_python | `perception_node`, `navigator_node`, `auto_target_generator`, launch, config |

## Prerequisite: the `umiusi_perception` wheel
The nodes import **only** `umiusi_perception` — the detector/tracker/FSM (needs **torch**) *and* the
feed-forward allocation `umiusi_perception.control` (pure **numpy**). This is the repo's on-robot
execution wheel (`packages/perception` in the `umiusi_sim` workspace at `../../umiusi_sim`): it contains
**no simulator, no RL training code, and no mujoco**. Install just that wheel into the environment ROS
runs in, e.g. on the Pi:

```bash
pip install '/path/to/umiusi_sim/packages/perception'       # or: pip install -e '/path/to/umiusi_sim/packages/perception'
```

The heavy imports are **deferred to the first frame**, so `colcon build` and `--help` do not need
torch; only actually running the nodes does.

## Build
```bash
cd ros2_ws
colcon build --packages-select umiusi_autonomy_msgs umiusi_autonomy
source install/setup.bash
```

## Run
```bash
ros2 launch umiusi_autonomy autonomy.launch.py            # 同梱の camp_real2.pt を使う
ros2 launch umiusi_autonomy autonomy.launch.py model_path:=/abs/path/to/detector.pt
# FSM-only dry run (no thruster commands):
ros2 launch umiusi_autonomy autonomy.launch.py publish:=false
# sim / 別の image publisher から画像をもらうとき (実機カメラのブリッジを起動しない):
ros2 launch umiusi_autonomy autonomy.launch.py use_camera_bridge:=false
```

実機カメラ (gst_camera_node) は RTSP に流すだけで ROS トピックを出さないので、`camera_bridge_node`
が要る。**以前この launch にはブリッジが無く、実機では画像が 1 枚も来ずに FSM が SEARCH から
出られなかった** (8/25 の水中 run)。いまは既定で起動する。

Parameters live in `config/autonomy.yaml` (topics, rates, camera FOV, calibration). `model_path`,
`image_topic`, `publish`, `max_duty`, `use_camera_bridge`, `rtsp_url` are also launch arguments.

### RL attitude control + keyboard teleop
Moved to the **`umiusi_rl_control`** package (builds independently of perception) — see
`../umiusi_rl_control/README.md` for `rl_attitude_node`, `teleop_keyboard`, and the `AttitudeTarget`
setpoint.

**Disarm / e-stop of `navigator_node`** (reuses `umiusi_rl_control`'s helper): publish `~/estop`
(`std_msgs/Bool` `true`) or call `~/arm` (`std_srvs/SetBool` `data: false`) to DISARM — the node stops
and asserts a **detach** every tick (`ThrusterOutput` `runnable.esc=servo=false` + zero, so the control
stack releases esc/servo). Re-arm with `~/arm` `data: true` (or `~/estop` `false`). Launch disarmed with
`start_armed:=false`. (Core's power-off / Standby mode is the other, stack-wide stop.)

## Deploy calibration (verify on hardware — cannot be inferred from the sim)
* **`fovy_deg`** (both nodes) must match the physical camera vertical FOV — it sets every
  bearing/range estimate.
* **IMU yaw rate**: `sensor_msgs/Imu.angular_velocity` is **rad/s** (ROS standard), matching the FSM.
  `yaw_rate_axis` (`x`/`y`/`z`, default `z`=up, the REP-103 body frame) and `yaw_rate_sign` select and
  orient the component. Confirm against the mounted IMU.
* **Servo scaling**: settled — `ThrusterOutput.angle` is commanded in **degrees** by `rl_attitude_node`
  (see `../umiusi_rl_control/README.md`); `servo_range_deg` (default 90, matching `configs/umiusi.yaml`)
  sets the half-range for the normalised-servo → degrees mapping.

## Test against the simulation
The navigator's thruster topics/types match `umiusi_sim_bridge`, so you can close the loop against
the MuJoCo sim without hardware: launch the bridge sim, feed it a rendered camera stream on
`image_topic`, and run this launch file. (Bridging the sim's rendered camera onto a ROS `Image`
topic is a small follow-up; today the full perception→navigation loop is validated headless by
`tools/autonomy_run.py`, which drives the identical `BalloonBehavior`.)
```
