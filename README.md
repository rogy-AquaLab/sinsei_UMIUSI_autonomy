# sinsei_UMIUSI_autonomy

Experimental UMIUSI autonomy + low-level control, as a **multi-package colcon repo** (one repo, split
packages so the control layer builds independently of perception).

| package | build type | role | key deps |
|---|---|---|---|
| [`umiusi_autonomy`](umiusi_autonomy/) | ament_python | perception + planner: `perception_node`, `navigator_node`, on-core `auto_target_generator` | `umiusi_perception` wheel (pip), `umiusi_rl_control` (arm helper) |
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
