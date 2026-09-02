# sinsei_UMIUSI_autonomy

Autonomy and low-level control for the UMIUSI underwater robot.
One colcon repo, four packages, so the control layer builds without perception.

| package | type | what it does |
|---|---|---|
| [`umiusi_autonomy`](umiusi_autonomy/) | ament_python | camera bridge, balloon detection, planner |
| [`umiusi_rl_control`](umiusi_rl_control/) | ament_python | RL attitude controller, keyboard teleop, arm / e-stop |
| [`umiusi_autonomy_msgs`](umiusi_autonomy_msgs/) | ament_cmake | detection messages |
| [`umiusi_rl_control_msgs`](umiusi_rl_control_msgs/) | ament_cmake | `AttitudeTarget` setpoint |

`umiusi_autonomy` depends on `umiusi_rl_control`, never the other way round.

## Setup

```bash
cd ~/ros2-ws/src
git clone https://github.com/rogy-AquaLab/sinsei_UMIUSI_autonomy.git
cd sinsei_UMIUSI_autonomy
./tools/setup_robot.sh          # deps into ~/.local, then colcon build

source ~/ros2-ws/install/setup.bash
```

`setup_robot.sh` does not touch system files. Python packages go to `~/.local`.
Only `rosdep` uses `sudo apt`, for ROS packages.

To get a Pi to the point where this works (ssh, CAN, mDNS, fixed IP), see
[`docs/robot_setup.md`](docs/robot_setup.md).

Build only the control layer (no perception, no torch for detection):

```bash
colcon build --packages-up-to umiusi_rl_control
```

## Run

Open two terminals (tmux is fine).

**Terminal 1 — teleop.** Start this first. It sends targets and it is the e-stop.

```bash
ros2 run umiusi_rl_control teleop_keyboard
```

| key | what happens |
|---|---|
| `w` `s` `a` `d` `r` `f` | velocity, body frame |
| `i` `k` `j` `l` `u` `o` | target attitude: pitch, yaw, roll |
| `SPACE` | velocity to zero, keep holding attitude |
| `t` | reset target attitude to upright |
| `z` | arm |
| `x` | e-stop: disarm and detach the thrusters |
| `q` | quit (disarms first) |

**Terminal 2 — the robot.** Start without moving the thrusters:

```bash
ros2 launch umiusi_autonomy stack.launch.py mode:=attitude publish:=false
```

Watch terminal 1. Tilt the robot by hand and check that the target attitude follows.
When that looks right, restart without `publish:=false` and press `z` in terminal 1 to arm.

Other modes:

```bash
ros2 launch umiusi_autonomy stack.launch.py                    # everything: control + perception + RL
ros2 launch umiusi_autonomy stack.launch.py mode:=perception   # control + camera + detection, no RL
```

The launch starts the hardware first, waits until `/state/imu` arrives, then starts the rest.
It does not start anything before the previous stage is up.

Useful arguments:

| argument | default | what it does |
|---|---|---|
| `mode` | `full` | `full` / `attitude` / `perception` |
| `publish` | `true` | `false` computes without commanding the thrusters |
| `use_control` | `true` | `false` if you already run the hardware or the sim bridge |
| `model_path` | bundled | detector checkpoint |
| `use_ui` | `true` | `false` skips rosbridge and frees CPU |

## Record a run

```bash
./tools/record_run.sh --name pool-01
```

Start it before the robot. It picks up topics that appear later.
Press `Ctrl-C` to stop the bag and the video together.

## When something does not work

Read [`docs/known_issues.md`](docs/known_issues.md). Every entry has an id, so code comments
can point at it (`known_issues A-11`).

## For developers

Calibration, diagnostics and offline analysis live in [`tools/`](tools/README.md).
You do not need any of them to run the robot.

Policy bundles, observation layout and the wrench-mode action contract are described in
[`umiusi_rl_control/README.md`](umiusi_rl_control/README.md).
Experiment procedures are in [`docs/experiment_guide.md`](docs/experiment_guide.md).
Recording and log layout are in [`docs/logging.md`](docs/logging.md).

Two combinations must never run at the same time:

- `sinsei_umiusi_core/launch/main.yaml` and this repo's full mode.
  Both start `auto_target_generator`, and the two fight over `/cmd/target`.
- `autonomy.launch.py` and `core_autonomy.launch.py`.
  The first drives the thrusters through `/cmd/direct`, the second through core's
  behaviour tree and `/cmd/target`. Two command paths reach the thrusters at once.
