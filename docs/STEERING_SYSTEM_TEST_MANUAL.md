# Steering Motor System — Operation & Testing Manual

**Repository:** `steering_motor_sys`  
**System:** Distributed Four-Motor Steering Control for an Agricultural Weeding Robot  
**Platform:** Ubuntu 22.04 · ROS 2 Humble · Raspberry Pi · STM32F407 · CANopen  
**Manual version:** 1.0  
**Last updated:** 2026-11-12  
**Validation status:** Laboratory startup, discovery, position feedback, and managed software shutdown verified

> **Safety notice:** This is a research prototype, not a safety-rated motion-control system. Software shutdown, ROS 2 `stop` messages, and terminating a bridge **do not constitute an emergency stop**. A hardware-level independent stopping/isolation method is required for supervised motion testing. Do not use this procedure for unattended operation. A network-loss fail-safe has **not** been verified.

## 1. Purpose and scope

This manual lets another laboratory team member operate the **currently deployed** NUC + two-Raspberry-Pi steering control system without launching three Python processes manually. It covers system prerequisites, startup, health checks, read-only motor feedback, supervised motion-test interfaces, troubleshooting, log inspection, and controlled software shutdown.

It distinguishes **verified behavior** from **command acceptance** and **future work**. In particular, accepting a target angle does not prove that the output shaft reached the mechanically measured angle.

### Verified system capabilities

| Capability | Evidence / status |
|---|---|
| NUC ↔ Pi A and NUC ↔ Pi B over Wi-Fi / ROS 2 DDS | Verified |
| Three ROS 2 nodes discovered from NUC | Verified |
| Positions returned from M1–M4 through their STM32/CANopen paths | Verified |
| Side A M1/M2 +90° → 0° positioning with encoder feedback | Verified in prior testing |
| Side B M3/M4 +90° → 0° positioning with encoder feedback | Verified |
| Four-motor commands routed to Side A and Side B | Verified |
| Independent four-motor speed settings routed by V5 | Verified as routing/acceptance; physical speed accuracy not calibrated |
| `start_steering.sh` starts remote and local managed sessions | Verified |
| `status_steering.sh` detects running and stopped states | Verified |
| `stop_steering.sh` closes managed tmux sessions and processes | Verified for the tested shutdown cycle |
| Deterministic four-motor synchronization | **Not verified** |
| NUC/Pi wireless-loss safe stop | **Not implemented/verified as a safety function** |
| Independent output-shaft angle calibration | **Not verified** |

## 2. Architecture

```text
                       NUC (Ubuntu 22.04 / ROS 2 Humble)
                         /four_motor_router_v5
                         /steering/four/command
                                   |
                           Wi-Fi / ROS 2 DDS
                         +---------+---------+
                         |                   |
                  Raspberry Pi A       Raspberry Pi B
                  192.168.50.242       192.168.50.228
                  /bridge_side_a       /bridge_side_b
                   bridge_sidea.py      bridge_sideb.py
                         |                   |
                      USB-UART            USB-UART
                         |                   |
                      STM32 A             STM32 B
                         |                   |
                      CANopen             CANopen
                       /   \               /   \
                      M1   M2             M3   M4
                     ID 1 ID 2           ID 3 ID 4
```

### Machine access

| Machine | SSH identity | Current IP | Responsibility |
|---|---|---|---|
| NUC | Local Linux user `me813` | Depends on network | V5 router, management scripts |
| Pi A | `ubuntu@192.168.50.242` | `192.168.50.242` | Global M1, M2 |
| Pi B | `me813@192.168.50.228` | `192.168.50.228` | Global M3, M4 |

These are **current laboratory addresses**, not universally assigned addresses. Confirm connectivity if the laboratory Wi-Fi setup changes. Both Pis have been configured for SSH public-key login from the existing NUC.

### Motor numbering

| Global name | Pi | STM32 UART-local motor | CANopen node ID |
|---|---|---|---|
| M1 | Pi A | M1 | 1 |
| M2 | Pi A | M2 | 2 |
| M3 | Pi B | M1 | 3 |
| M4 | Pi B | M2 | 4 |

Local `motors=[1,2]` in a Side B bridge log can be correct: the V5 router translates the *global* M3/M4 addresses to the bridge's local M1/M2 commands.

## 3. Repository and installed paths

**NUC canonical workspace:**

```text
/home/me813/steering_motor_sys/
├── README.md
├── docs/
│   └── STEERING_SYSTEM_TEST_MANUAL.md   # this file
├── scripts/
│   ├── start_steering.sh
│   ├── status_steering.sh
│   └── stop_steering.sh
├── matlab_simulation/
└── src/
    └── dual_steering_motor_bridge/
        ├── bridge_sidea.py
        ├── bridge_sideb.py
        ├── dual_steering_ros2_bridge_V5.py
        └── legacy/
```

An **NUC-only compatibility symlink** currently exists:

```text
/home/me813/Dual_Steering_Motor_Test -> /home/me813/steering_motor_sys
```

**Important:** The two Pis still deploy their bridges at the **old remote directory path**:

```text
~/Dual_Steering_Motor_Test/src/dual_steering_motor_bridge/
```

The `start_steering.sh` uses the NUC repository-relative path for V5, while the Pi bridge locations intentionally retain their existing paths. Do **not** globally replace the old directory name in scripts without checking remote deployment locations.

Repository URL: <https://github.com/n16145014-cmd/steering_motor_sys>

## 4. Before every test: hardware safety and readiness

1. Put the robot/steering assembly in a secured test configuration; prevent unexpected vehicle movement.
2. Ensure that all four wheel forks can move without mechanical collision and that the commanded angular limits have been checked.
3. Confirm nobody is in the motion envelope and that electrical connections are secured.
4. Confirm the **independent hardware stopping/isolation method** is present, tested and accessible. Understand what it does to motor power and brakes.
5. Confirm any prior motion has completed before changing software lifecycle state.
6. Power the NUC, both Pis, STM32 boards and motor drivers as appropriate for the intended communication test; confirm all equipment is on the intended network.

**Never assume:** `REF_VALID=1` proves mechanical zero, `MOVING=0` proves torque is removed, or ROS node shutdown puts the drive into a safe state.

## 5. Start the system from the NUC

Open a terminal **on the NUC**:

```bash
cd ~/steering_motor_sys
./scripts/start_steering.sh
```

Example of a successful launcher run:

```text
=== Starting Steering System ===
Checking ubuntu@192.168.50.242 ...
Started side_a
Checking me813@192.168.50.228 ...
Started side_b
Starting V5 on NUC ...
=== Startup commands completed ===
Verify ROS 2 nodes and motor status before use.
```

The script starts Pi A and Pi B bridges over SSH inside named `tmux` sessions and starts the NUC V5 router in a local `tmux` session. It **does not** run `sethome`, `enable`, or motor-angle commands automatically.

**Expected managed sessions:**

| Host | tmux session | Process |
|---|---|---|
| Pi A | `steering_bridge` | `bridge_sidea.py` |
| Pi B | `steering_bridge` | `bridge_sideb.py` |
| NUC | `steering_v5` | `dual_steering_ros2_bridge_V5.py` |

A launcher success message means the managed sessions were started; complete the following checks to verify system connectivity.

## 6. Check system status

On the NUC:

```bash
cd ~/steering_motor_sys
./scripts/status_steering.sh
```

**Expected running-state indicators:**

```text
===== RASPBERRY PI A - M1/M2 =====
[OK] SSH connected
[OK] tmux steering_bridge exists
[OK] Bridge process running
[INFO] UART is in use

===== RASPBERRY PI B - M3/M4 =====
[OK] SSH connected
[OK] tmux steering_bridge exists
[OK] Bridge process running
[INFO] UART is in use

===== NUC - V5 ROUTER =====
[OK] tmux steering_v5 exists
[OK] V5 Router process running

===== ROS 2 NODES =====
[OK] /bridge_side_a
[OK] /bridge_side_b
[OK] /four_motor_router_v5
```

`[OK]` identifies an existing process/session/node, **not** an independent proof that the motor driver or brake is safe or responsive.

### ROS 2 CLI environment

Run these lines in **each new NUC terminal** used for ROS commands:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=30
export ROS_LOCALHOST_ONLY=0
```

Check nodes and topics:

```bash
ros2 node list
ros2 topic list
```

The required nodes are:

```text
/bridge_side_a
/bridge_side_b
/four_motor_router_v5
```

Key topics:

| Topic | Purpose |
|---|---|
| `/steering/four/command` | Unified four-motor command input; `std_msgs/msg/String` |
| `/steering/four/result` | Router and per-side command results |
| `/steering/four/status` | Aggregated per-motor status / position feedback |
| `/steering/side_a/motor/command` | Pi A bridge commands |
| `/steering/side_b/motor/command` | Pi B bridge commands |

## 7. First functional test: read all four positions

**This is the recommended non-motion test after startup.**

In NUC Terminal 1 (after sourcing ROS as above):

```bash
ros2 topic echo /steering/four/status
```

In NUC Terminal 2 (after sourcing ROS as above):

```bash
ros2 topic pub --once /steering/four/command \
  std_msgs/msg/String "{data: 'all;position'}"
```

Look for a genuine `POS` message from **each of M1, M2, M3, M4**, not merely `ROUTED`, `QUEUED`, or `OK COMMAND COMPLETED`.

**Example of previously recorded feedback** (actual positions can differ between tests):

```text
side_a: OK M1 POS ACT_POS=1158 REL_STEPS=1 REF_VALID=1 MOVING=0
side_a: OK M2 POS ACT_POS=472 REL_STEPS=0 REF_VALID=1 MOVING=0
side_b: OK M3 POS ACT_POS=-2977 REL_STEPS=0 REF_VALID=1 MOVING=0
side_b: OK M4 POS ACT_POS=3069 REL_STEPS=0 REF_VALID=1 MOVING=0
```

| Field | Meaning |
|---|---|
| `ACT_POS` | Current motor/driver position in internal encoder units |
| `REL_STEPS` | Position relative to the current software zero reference |
| `REF_VALID` | 1 = software zero reference valid; 0 = no valid reference |
| `MOVING` | Controller-reported movement flag |

**Interpretation:** `REF_VALID=0` is **not necessarily a communication fault**. A reference must only be set after the operator physically checks the intended mechanical zero and the firmware's preconditions. The small `REL_STEPS` variations recorded previously were within a few encoder steps and do not establish mechanical angular accuracy.

## 8. Command interface and supervised motion testing

> **Operator restriction:** Do not send the commands in this section unless the steering assembly is secured, the full rotation envelope is verified, a suitable independent hardware stop is functional, the correct mechanical zero has been established, and the drives are in their intended state. First-time operators should conduct the read-only position test under supervision and not proceed directly to a 90° movement.

### 8.1 Reference and enable concepts

The system supports these **command strings**:

```text
all;sethome
all;enable
```

- `sethome` sets a **volatile software** position reference; it does not search for a physical home switch or verify mechanical alignment.
- `enable` requests motor drive enable. A message such as `OK ENABLE VERIFIED ...` refers to **CiA402 status**, not independently sensed physical brake engagement.
- Whether an operation is permitted depends on the existing STM32 firmware conditions. Check status and actual hardware state at every step.

These examples document the protocol; they are **not part of the normal startup script**.

### 8.2 Four-motor angle / speed command

The central V5 command structure is:

```text
all;A[M1_angle,M2_angle,M3_angle,M4_angle];S[M1_speed,M2_speed,M3_speed,M4_speed]
```

The `A` array is in commanded degrees; the `S` array is interpreted by the current motor firmware/driver configuration. Routing to both Pis does **not** imply deterministic simultaneous arrival or motion start.

**Small-angle bench-test example, only after all safety prerequisites are met:**

```bash
ros2 topic pub --once /steering/four/command \
  std_msgs/msg/String \
  "{data: 'all;A[5,5,5,5];S[50,50,50,50]'}"
```

Confirm physical direction, clearance, encoder movement and final `MOVING=0` for all motors. If and only if the movement has completed and returning is safe:

```bash
ros2 topic pub --once /steering/four/command \
  std_msgs/msg/String \
  "{data: 'all;A[0,0,0,0];S[50,50,50,50]'}"
```

Historical development tests also sent +90°, −90°, and 0° commands and routed different speed settings per motor. **Do not use ±90° as a default first-test instruction.**

### 8.3 Position conversion and validation

The current firmware tests have used an assumed conversion of **20,000 encoder steps per commanded degree**:

| Command angle | Approximate relative encoder steps |
|---|---:|
| 0° | 0 |
| +5° | +100,000 |
| +90° | +1,800,000 |
| −90° | −1,800,000 |

This conversion is the current **software assumption** and must be checked against the actual gearbox/output steering axis before claims about physical angle accuracy.

### 8.4 Monitor command acceptance vs movement completion

Terminal 1:

```bash
ros2 topic echo /steering/four/result
```

Terminal 2:

```bash
ros2 topic echo /steering/four/status
```

A message such as:

```text
ROUTED side_b: all;A[90,90];S[500,500]
side_b: QUEUED ABSOLUTE motors=[1, 2]
side_b: OK COMMAND COMPLETED (accepted; motion may still be running)
```

**does not prove arrival at the requested angle**. Confirm feedback such as `OK M3 REACHED ...`, `REL_STEPS` near target, and `MOVING=0`, plus physical observation or independent angle measurement when required.

## 9. View live logs without interrupting a running bridge

From the NUC:

**Pi A:**

```bash
ssh -t ubuntu@192.168.50.242 'tmux attach -t steering_bridge'
```

**Pi B:**

```bash
ssh -t me813@192.168.50.228 'tmux attach -t steering_bridge'
```

**NUC V5 Router:**

```bash
tmux attach -t steering_v5
```

To disconnect from a `tmux` session while keeping the process running: **Ctrl+B, then D**. Do not use Ctrl+C just to leave a log view.

## 10. Managed software shutdown

**Prerequisites:** All four motors physically stationary, the drives in an approved safe state, and the laboratory's independent hardware stop/isolation procedure completed as needed. Software shutdown alone does **not** stop torque or provide a safety-rated condition.

On the NUC:

```bash
cd ~/steering_motor_sys
./scripts/stop_steering.sh
```

The script asks:

```text
Press ENTER to continue (請確認實體急停已啟用)...
```

Press **Enter only after personally confirming the stated safety condition**. The script does not activate the physical stop itself.

Expected result when all managed sessions are running:

```text
===== Stopping NUC V5 Router =====
[OK] V5 tmux session stopped

===== Stopping RASPBERRY PI A =====
[OK] Bridge tmux session stopped

===== Stopping RASPBERRY PI B =====
[OK] Bridge tmux session stopped
```

Run a follow-up check:

```bash
./scripts/status_steering.sh
```

In the stopped state it is **expected** to report:

```text
[WARN] tmux steering_bridge missing
[WARN] Bridge process not running
[INFO] UART is not in use
[WARN] tmux steering_v5 missing
[WARN] V5 Router process not running
```

And `/bridge_side_a`, `/bridge_side_b`, and `/four_motor_router_v5` should no longer be discovered. `stop_steering.sh` manages only the named tmux sessions; a separately started process may still exist and must be investigated if status shows a discrepancy.

## 11. Troubleshooting

| Symptom | Likely cause / next check |
|---|---|
| `ERROR: UART already in use` | Existing bridge or other serial process owns the Pi's `/dev/ttyUSB0`. Identify its PID before stopping it. |
| `Bridge process not running` | Inspect the relevant tmux session/log; verify Python file, serial device, ROS environment. |
| SSH login fails | Verify IP, Wi-Fi, Pi power, SSH service, and authorized key. |
| ROS 2 node absent | Check bridge/router processes and `ROS_DOMAIN_ID=30`, `ROS_LOCALHOST_ONLY=0` on all devices. |
| `ROUTED` appears but no motor `POS` | Inspect per-side bridge output, UART connection, STM32, driver power and CANopen node mapping. |
| `REF_VALID=0` | No valid software reference. Re-establish only after verified mechanical alignment and approved procedure. |
| `tmux session not found` during stop | Session was already absent. Use `status_steering.sh` to confirm no unmanaged process remains. |

**Read-only diagnostic commands from NUC:**

```bash
# Who owns the Pi A serial port?
ssh ubuntu@192.168.50.242 'fuser -v /dev/ttyUSB0 || true'

# Who owns the Pi B serial port?
ssh me813@192.168.50.228 'fuser -v /dev/ttyUSB0 || true'

# Check that key-based SSH works
ssh -o BatchMode=yes ubuntu@192.168.50.242 hostname
ssh -o BatchMode=yes me813@192.168.50.228 hostname

# Check managed sessions
ssh ubuntu@192.168.50.242 'tmux ls'
ssh me813@192.168.50.228 'tmux ls'
tmux ls
```

Never use `kill -9`, `fuser -k`, or unplug a motor control connection as a routine stop procedure, especially during movement. Escalate abnormal hardware behavior using the approved independent emergency stopping procedure.

## 12. Standard test checklist

### Before starting

- [ ] Four steering mechanisms secured and movement range clear.
- [ ] Independent hardware stop/isolation method available and checked.
- [ ] NUC, Pi A, Pi B, STM32s and required drive power ready.
- [ ] Lab Wi-Fi connectivity and current IP addresses confirmed.

### Startup and communication

- [ ] Run `./scripts/start_steering.sh` on NUC.
- [ ] Run `./scripts/status_steering.sh`; all three expected ROS 2 nodes found.
- [ ] Send read-only `all;position` query.
- [ ] Confirm **four** valid motor position responses M1–M4.
- [ ] Record `REF_VALID`, `REL_STEPS`, `MOVING` for each motor.

### Supervised motion testing (optional; separately authorized)

- [ ] Mechanical zero alignment independently verified.
- [ ] Firmware reference and motor enable states verified.
- [ ] Limits, commanded direction, speed, and physical clearance checked.
- [ ] Begin with a small safe motion; monitor feedback and physical movement.
- [ ] Verify target reached and `MOVING=0` before any follow-up movement.

### End of test

- [ ] Motors physically stopped and hardware-safe procedure completed.
- [ ] Run `./scripts/stop_steering.sh` and confirm the prompt only when true.
- [ ] Run `./scripts/status_steering.sh`; no managed nodes/processes found; UART released.
- [ ] Record anomalies in the lab test log.

## 13. Quick reference

From NUC, enter the workspace first:

```bash
cd ~/steering_motor_sys
```

| Operation | Command |
|---|---|
| Start managed system | `./scripts/start_steering.sh` |
| Check managed system | `./scripts/status_steering.sh` |
| Stop managed software (after hardware safety checks) | `./scripts/stop_steering.sh` |
| Show ROS 2 nodes | `ros2 node list` |
| Show per-command results | `ros2 topic echo /steering/four/result` |
| Show motor positions/status | `ros2 topic echo /steering/four/status` |
| Request all positions | `ros2 topic pub --once /steering/four/command std_msgs/msg/String "{data: 'all;position'}"` |

For all ROS CLI commands, source `/opt/ros/humble/setup.bash` and export `ROS_DOMAIN_ID=30`, `ROS_LOCALHOST_ONLY=0` in that terminal first.

**Standard operating sequence:**

```text
Hardware safety checks
        ↓
start_steering.sh
        ↓
status_steering.sh
        ↓
all;position → verify M1–M4 feedback
        ↓
Optional separately approved supervised motion test
        ↓
Confirm all motors stationary / hardware safe state
        ↓
stop_steering.sh
        ↓
status_steering.sh → verify software OFF
```

## 14. Remaining engineering work

- Hardware-level safety architecture and verified network-loss handling.
- Output-shaft zero/angle calibration and travel-limit validation.
- Deterministic multi-motor coordination (if required by controller design).
- Integration of four-wheel steering kinematics with the ROS 2 command interface.
- Configuration portability (move fixed Pi IPs and serial paths to maintained configuration).
- More robust health checks, diagnostic logging, failure recovery and restart testing.

---

**Manual status:** This document reflects the laboratory setup and software tests reported on **2026-11-12**. Review IP addresses, firmware, hardware condition, scripts, and safety provisions before using it on a modified robot.
