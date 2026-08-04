# umiusi_autonomy — deploy-side perception + navigation nodes

Two thin rclpy nodes that run the **same** ROS-free perception + autonomy code as the in-sim run
(`umiusi_sim/tools/autonomy_run.py`), on the real robot:

```
 onboard camera            perception_node                 navigator_node                sinsei_umiusi_control
 sensor_msgs/Image  ─────▶ learned detector +      ─────▶  behaviour FSM +        ─────▶ /cmd/direct/thruster_
 (/front_cam/image_raw)    sanitise_near_colours          feedforward_allocation         controller/output_{lf,lb,rb,rf}
                           │                               ▲  ImuState (yaw rate)         (ThrusterOutput, direct override)
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
* **`rl_attitude_node`** — a **self-contained** RL attitude(-velocity) controller: the trained
  policy (bundled in `models/cruise_policy`) driving the thrusters directly. A rclpy port of
  `umiusi_sim/tools/ros_policy.py` that needs **no umiusi_sim / umiusi_rl / mujoco** — only
  stable-baselines3 + torch + gymnasium. Subscribes `/state/imu_state` + `/state/thruster_state_all`,
  rebuilds the 25-D `attitude_velocity`/`imu` observation (verified bit-identical to
  `UmiusiPoseEnv._get_obs`), and publishes the four `/cmd/direct/...` `ThrusterOutput`. The target
  attitude + velocity are set in **real time** via topics (`geometry_msgs/Quaternion` +
  `Vector3`; default = upright + cruise), so a teleop/joystick controller just publishes them. Launch
  it standalone with `launch/rl_attitude.launch.py`. See its module docstring for the deg/rad unit
  caveats (inherited from `ros_policy`).

All detection/decision/allocation logic lives in the installable `umiusi_perception` package (detector +
FSM + the numpy-only `umiusi_perception.control` allocation); these nodes only do topic plumbing + message
conversion. That is why the perception and behaviour are bit-identical between simulation and the robot.

## Packages
| package | build type | contents |
|---|---|---|
| `umiusi_autonomy_msgs` | ament_cmake (rosidl) | `BalloonDetection`, `BalloonDetectionArray` |
| `umiusi_autonomy` | ament_python | `perception_node`, `navigator_node`, `auto_target_generator`, `rl_attitude_node`, `teleop_keyboard`, launch, config, bundled policy |

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
ros2 launch umiusi_autonomy autonomy.launch.py model_path:=/abs/path/to/detector.pt
# FSM-only dry run (no thruster commands):
ros2 launch umiusi_autonomy autonomy.launch.py model_path:=/abs/model.pt publish:=false
```

Parameters live in `config/autonomy.yaml` (topics, rates, camera FOV, calibration). `model_path`,
`image_topic`, and `publish` are also launch arguments.

### RL attitude control only (no perception / FSM)
Drive the thrusters with the bundled trained policy (hold upright + cruise +X):
```bash
ros2 launch umiusi_autonomy rl_attitude.launch.py                 # bundled cruise policy
ros2 launch umiusi_autonomy rl_attitude.launch.py vel_cmd:=0.3    # slower cruise
ros2 launch umiusi_autonomy rl_attitude.launch.py publish:=false  # predict only (no thrusters)
```
**Real-time setpoints** (last message wins; defaults = upright + `vel_cmd`): publish the target
attitude and/or velocity — a teleop / joystick controller just publishes these:
```bash
# target attitude (geometry_msgs/Quaternion, x y z w) — here: identity = upright
ros2 topic pub /rl_attitude_node/target_attitude geometry_msgs/msg/Quaternion "{x: 0, y: 0, z: 0, w: 1}"
# target velocity in the target-body frame (geometry_msgs/Vector3) — cruise +X at 0.3 m/s
ros2 topic pub /rl_attitude_node/velocity_cmd geometry_msgs/msg/Vector3 "{x: 0.3, y: 0, z: 0}"
```
Topic names are the `attitude_topic` / `velocity_topic` params (default `~/target_attitude`,
`~/velocity_cmd`). Needs `stable-baselines3`, `torch`, `gymnasium` in the ROS runtime env (not rosdep
keys — install with pip, like the `umiusi_perception` wheel). Requires the controllers/bridge
providing `/state/imu_state` + `/state/thruster_state_all`.

### Keyboard teleop + emergency stop
`teleop_keyboard` drives `rl_attitude_node` from the keyboard (3-D: separate keys per axis), for
experiments. Run it in its **own terminal** (it needs the keyboard):
```bash
ros2 run umiusi_autonomy teleop_keyboard
```
`w/s a/d r/f` = ±velocity x/y/z (body), `i/k j/l u/o` = pitch/yaw/roll target, `SPACE` = zero velocity,
`t` = upright, **`x` = EMERGENCY STOP**, `z` = re-arm, `q` = quit. The e-stop both signals the
controller to disarm **and** directly detaches the thrusters (independent of the controller).

**Disarm / e-stop (all direct-drive autonomy nodes** — `rl_attitude_node`, `navigator_node`): publish
`~/estop` (`std_msgs/Bool` `true`) or call `~/arm` (`std_srvs/SetBool` `data: false`) to DISARM — the
node stops and asserts a **detach** every tick (`ThrusterOutput` `runnable.esc=servo=false` + zero, so
the control stack releases esc/servo). Re-arm with `~/arm` `data: true` (or `~/estop` `false`). Launch
disarmed with `start_armed:=false`. (Core's power-off / Standby mode is the other, stack-wide stop.)

## Deploy calibration (verify on hardware — cannot be inferred from the sim)
* **`fovy_deg`** (both nodes) must match the physical camera vertical FOV — it sets every
  bearing/range estimate.
* **IMU yaw rate**: `ImuState.angular_velocity` is **deg/s**; the FSM wants the body yaw rate in
  rad/s. `yaw_rate_axis` (`x`/`y`/`z`, default `y`=up as in sim) and `yaw_rate_sign` select and
  orient the component. Confirm against the mounted IMU.
* **Servo scaling**: `ThrusterOutput.angle` is documented in **rad**; `servo_range_deg` (default 90,
  matching `configs/umiusi.yaml`) sets the half-range for the normalised-servo → rad mapping. NOTE:
  `tools/ros_policy.py` currently scales in degrees — reconcile the two against the live bridge
  during bring-up (the spec's open *FF-frame sign reconcile* item).

## Test against the simulation
The navigator's thruster topics/types match `umiusi_sim_bridge`, so you can close the loop against
the MuJoCo sim without hardware: launch the bridge sim, feed it a rendered camera stream on
`image_topic`, and run this launch file. (Bridging the sim's rendered camera onto a ROS `Image`
topic is a small follow-up; today the full perception→navigation loop is validated headless by
`tools/autonomy_run.py`, which drives the identical `BalloonBehavior`.)
```
