# Distributed four-motor steering control

This repository contains bench-tested steering control software and MATLAB research models for a four-wheel independently steered agricultural weeding robot. The active path uses ROS 2 Humble to route commands from a NUC to two Raspberry Pis. Each Pi talks to one STM32F407 over USB-UART; each STM32 controls two Oriental Motor steering drivers over CANopen. Drive-wheel control and autonomous field operation are outside the verified scope.

## Architecture and motor mapping

```text
NUC (Ubuntu 22.04, ROS 2 Humble)
  dual_steering_ros2_bridge_V5.py  /four_motor_router_v5
             | DDS over Wi-Fi, /steering/side_*/motor/command
       +-----+-----+
       |           |
Raspberry Pi A   Raspberry Pi B
bridge_sidea.py  bridge_sideb.py -> imports bridge_sidea.py
       |           |
   USB-UART     USB-UART (115200 baud)
       |           |
   STM32 A      STM32 B
       |           |
   CANopen      CANopen
   Nodes 1, 2   Nodes 3, 4
```

| Global motor | Pi / bridge | Bridge local motor | CANopen node |
| --- | --- | --- | --- |
| `m1` | A / `side_a` | `m1` | 1 |
| `m2` | A / `side_a` | `m2` | 2 |
| `m3` | B / `side_b` | `m1` | 3 |
| `m4` | B / `side_b` | `m2` | 4 |

The Side B UART protocol still uses local `M1` and `M2`. V5 maps global IDs to those local IDs. Node IDs do not, by themselves, establish front/rear or left/right wheel geometry. Keep `bridge_sidea.py` and `bridge_sideb.py` together: Side B imports the shared implementation from Side A.

## Repository layout

```text
docs/                              Current notes and historical V3 manual
docs/legacy/                       Historical V3 command reference
scripts/start_steering.sh          NUC launcher for both Pis and V5
src/dual_steering_motor_bridge/
  bridge_sidea.py                  Shared bridge and Side A entry point
  bridge_sideb.py                  Side B entry point
  dual_steering_ros2_bridge_V5.py  Active four-motor router
  legacy/                          Earlier bridges and alternate V5 router
matlab_simulation/                  Kinematics and line-following experiments
  FarmSimulation/                  Field models, reusable functions, tests, video
  Legacy/                          Earlier passive row-change studies
```

This is a **hybrid research repository**, not a colcon package. There is no `package.xml`, `setup.py`, or `CMakeLists.txt`; run the Python entry points as scripts after sourcing ROS 2. `colcon build` is not required. The archived programs are retained for study and should not run alongside the active bridges or router because they may compete for UART devices or ROS topics. See [MATLAB simulation notes](matlab_simulation/README.md) and the [historical V3 manual](docs/Dual_Steering_ROS2_Bridge_V3_操作手冊.md).

## Requirements and startup

- Ubuntu 22.04 and ROS 2 Humble on the NUC, with compatible ROS 2 Humble environments on both Pis.
- Python 3 with `rclpy`, `std_msgs`, and `pyserial` (`serial` import) where the Python files run.
- SSH key access from the NUC to both Pis; `tmux` on all three machines; `fuser` on the Pis; ROS 2 DDS network reachability.
- Matching bridge entry points already installed on the Pis. This cleanup does not deploy to them.

The launcher has fixed Pi SSH addresses, a USB serial **by-id** path, the repository path under each user's home directory, and `ROS_DOMAIN_ID=30` with `ROS_LOCALHOST_ONLY=0`. Confirm these values in [start_steering.sh](scripts/start_steering.sh) against the actual machines before running it. Confirm the physical motor-to-node map and that no other process owns either UART. The launcher does not send motion, enable, or set-home commands.

From the NUC repository root, after checking hardware safety and the launcher settings:

```bash
bash scripts/start_steering.sh
```

The script uses noninteractive SSH, starts each bridge in a `steering_bridge` tmux session, then starts V5 on the NUC in `steering_v5`. An existing session is reported and reused; **its contents are not validated**. It refuses a new Pi session when `fuser` reports the configured UART in use, and refuses a NUC router when it finds a matching Python process outside tmux. If SSH, device checks, or startup fail, it exits; earlier successfully started sessions can remain running. Inspect actual tmux panes and processes before sending commands. Do not force-kill another UART owner merely to make startup pass.

## Inspect the live ROS 2 system

On a ROS 2 Humble terminal with the same DDS domain as the launcher:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=30 ROS_LOCALHOST_ONLY=0
ros2 node list
ros2 topic list -t
ros2 topic info /steering/four/command -v
ros2 topic info /steering/side_a/motor/command -v
ros2 topic info /steering/side_b/motor/command -v
```

Expected nodes are `/four_motor_router_v5`, `/bridge_side_a`, and `/bridge_side_b`. The primary command interface is `/steering/four/command` (`std_msgs/msg/String`). Router results and forwarded MCU lines appear on `/steering/four/result` and `/steering/four/status`. Each side also has `/steering/side_a/...` or `/steering/side_b/...` command, result, status, and feedback topics. Discovery alone does not prove the drivers are ready or enabled.

To monitor position feedback without commanding motion, use separate terminals for continuous echoes:

```bash
ros2 topic echo /steering/four/status
ros2 topic echo /steering/side_a/left_steering/feedback_deg
ros2 topic echo /steering/side_a/right_steering/feedback_deg
ros2 topic echo /steering/side_b/left_steering/feedback_deg
ros2 topic echo /steering/side_b/right_steering/feedback_deg
```

These `Float64` feedback topics report each bridge's *local* M1/M2 encoder-derived relative angle when the software reference is valid. After verifying one router and one bridge per side, start an echo in one terminal to capture a fresh read-only position transaction:

```bash
ros2 topic echo /steering/four/result
```

Then, in a second terminal:

```bash
ros2 topic pub --once /steering/four/command std_msgs/msg/String "{data: 'all;position'}"
```

The position response is reported through result and status topics. Angle feedback uses the bridge's assumed conversion of 20,000 position steps per degree; it is not an independent measurement of the wheel's mechanical angle.

## Verified state and limits

Previous bench tests reported successful independent Side A and Side B positioning, including +90° to 0° with encoder feedback. The NUC V5 router also routed four-motor commands to both Pis, and prior logs showed four motors returning to software zero. These are observations from earlier tests, not guarantees for a new startup or proof of field readiness.

- V5 routes commands; it does **not** guarantee synchronized starting times across the two Pis.
- A wireless-disconnection safety-stop mechanism for the distributed system has **not** been verified.
- `all;stop` requests a software Quick Stop. It is **not** a certified emergency stop. Use an independent hardware stopping method and keep the mechanism restrained and clear during tests.
- Encoder-based positioning has **not** established independently measured mechanical angular accuracy, including gearbox backlash or mounting error.
- `sethome` sets a temporary software reference at the current position. It is **not** mechanical homing; confirm the real zero and firmware preconditions before using it.
- A routed or accepted command is **not** proof that a motor reached its target. Check position, moving state, faults, and the physical mechanism.
- MATLAB ROS 2 experiments use their own topics and assumptions; they have not been integrated with the active V5 command interface. Do not run their motor mode against this distributed setup without reviewing the interface.

No hardware motion tests are part of repository maintenance. Future work includes independent angular calibration, network-loss behavior, cross-side fault handling, and measured start-time synchronization.

## Documentation

- [Steering System Operation & Testing Manual](docs/STEERING_SYSTEM_TEST_MANUAL.md)
