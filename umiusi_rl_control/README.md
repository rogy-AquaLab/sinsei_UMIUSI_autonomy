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
```
Requires the controllers/bridge (`sinsei_umiusi_control` or `umiusi_sim_bridge`) providing
`/state/imu` and consuming `/cmd/direct/...`.

**Policy bundles** (`models/`): `av_cal1_best_rep103` (default, 17-D attitude+velocity),
`att_cal1_best_rep103` (14-D attitude-only fallback), `av_sim2real2_rep103` (17-D, plan B).
All consume **REP-103 body-frame** observations (`export/meta.json` `obs_frame: rep103` is
enforced at load) and carry `golden.npz` sim-recorded obs→action vectors that the node replays
at load — a mismatch refuses to run (deploy-time verification, issue #15 A-5).

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
