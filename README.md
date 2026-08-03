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
  (`umiusi_perception.autonomy.BalloonBehavior`), and maps its `{surge, heave, yaw}` command through
  `umiusi_perception.control.feedforward_allocation` to the four **direct-override** `ThrusterOutput` topics
  — the identical drive path `tools/ros_policy.py` uses, so it drives the existing
  sinsei_umiusi_control stack **unchanged**.

All detection/decision/allocation logic lives in the installable `umiusi_perception` package (detector +
FSM + the numpy-only `umiusi_perception.control` allocation); these nodes only do topic plumbing + message
conversion. That is why the perception and behaviour are bit-identical between simulation and the robot.

## Packages
| package | build type | contents |
|---|---|---|
| `umiusi_autonomy_msgs` | ament_cmake (rosidl) | `BalloonDetection`, `BalloonDetectionArray` |
| `umiusi_autonomy` | ament_python | `perception_node`, `navigator_node`, launch, config |

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
