# Optimus — Biped Robot

Biped walking robot. The current goal is teaching it to walk. All active development is in `firmware/`.

## Hardware

| Component | Part | Interface |
|---|---|---|
| MCU | ESP32-C3 Super Mini | USB-CDC, single-core |
| Servo driver | PCA9685 16-ch PWM | I2C @ 0x40 |
| IMU | MPU6050 | I2C @ 0x68 |
| Leg servos (×8) | MG995 | PWM 600–2650 µs |
| Arm servos (×6 + hip yaw) | Futaba S3003 | PWM 900–2100 µs |
| I2C pins | SDA = GPIO 8, SCL = GPIO 9 | 400 kHz |

Leg geometry: femur L1=90mm → knee → tibia L2=90mm → ankle → foot L3=30mm.
CoM at hip ~250mm above ground when standing. Ankle joint 40mm above ground.

## Firmware Structure

```
firmware/
  firmware.ino        # Arduino entry point (setup/loop)
  src/
    main.cpp          # FreeRTOS task setup, shared globals, write_leg(), write_arms_tpose()
    gait/
      cpg.cpp/h       # Central Pattern Generator oscillator
      stabilizer.cpp/h # IMU-based gait stabilizer
      walk.cpp/h      # IMU-gated 4-phase walk state machine
    kinematics/
      forward.cpp/h   # FK: fk_solve() (real-time) + fk_full_dh() (verification)
      inverse.cpp/h   # IK: ik_solve(x,y,z) → JointAngles
    imu/
      mpu6050.cpp/h   # MPU6050 driver + complementary filter (α=0.98)
    servo/
      pca9685.cpp/h   # PCA9685 driver
    comms/
      telemetry.cpp/h # WiFi WebSocket server (port 81)
    experiments/
      exp_gait.cpp/h  # ENABLE_GAIT: open-loop CPG walk
      exp_walk.cpp/h  # ENABLE_WALK: IMU-gated walk
      exp_balance.cpp/h # ENABLE_BALANCE: static two-foot balance
      exp_slb.cpp/h   # ENABLE_SINGLE_LEG_BALANCE: right-foot single-leg balance
      exp_ik_sway.cpp/h # ENABLE_IK_SWAY: sagittal sway via IK (current default)
      shared.h        # extern globals, RAD2DEG/DEG2RAD, LEG_CH, SERVO_DIR tables
  include/
    config.h          # All tuning parameters and mode flags
```

## Build & Flash

```bash
cd optimus/firmware
make setup          # first time only: installs ESP32 core + WebSockets lib
make compile        # build
make compile-upload # build + flash
make monitor        # serial at 115200 baud
```

**Always run `make compile` (or the compile-check skill) after any edit to `src/` or `include/`.** Do not declare a task done until the build passes.

## Mode Flags (config.h)

Exactly **one** `ENABLE_*` flag active at a time — `static_assert` guards enforce mutual exclusivity.

| Flag | Current | Description |
|---|---|---|
| `ENABLE_IK_SWAY` | **1** | Sagittal sway via IK (current default) |
| `ENABLE_GAIT` | 0 | Open-loop CPG walk |
| `ENABLE_WALK` | 0 | IMU-gated 4-phase walk state machine |
| `ENABLE_BALANCE` | 0 | Static two-foot IMU balance |
| `ENABLE_SINGLE_LEG_BALANCE` | 0 | Balance on right foot only |
| `USE_IMU` | 1 | Enable MPU6050; required by WALK, BALANCE, SLB |
| `SERVO_SWEEP_TEST` | 0 | Sweep each leg channel ±20° on boot |

## Joint & Servo Layout

Each leg has **4 DOF**: hip_roll (θ1), hip_pitch (θ2), knee (θ3), ankle_roll (θ4).
**Ankle constraint**: θ4 = −θ1 always (keeps foot flat — critical, do not break this).

PCA9685 channel mapping:

| ch | Joint | `set_inverse` | Halt° |
|---|---|---|---|
| 0 | R ankle_roll | False | 90 |
| 1 | R knee | False | 80 |
| 2 | R hip_pitch | **True** | 100 |
| 3 | R hip_roll | **True** | 100 |
| 12 | L hip_roll | False | 80 |
| 13 | L hip_pitch | False | 90 |
| 14 | L knee | **True** | 90 |
| 15 | L ankle_roll | **True** | 70 |
| 4–9 | Arms (R/L shoulder FB, shoulder LAT, forearm LAT) | False | 90 |
| 10 | Hip yaw | False | 90 |

Servo output formula: `physical_deg = joint_rad × RAD2DEG × SERVO_DIR[ch] + SERVO_OFFSET_DEG[ch]`

To tune servo offsets: edit `SERVO_OFFSET_DEG_*` in `include/config.h`.

## CPG Gait (ENABLE_GAIT)

Sinusoidal oscillator per joint: `θᵢ(t) = Aᵢ · sin(ω·t + φᵢ) + θᵢ_offset`

Key phase relationships:
- L vs R hip_pitch: anti-phase (180°)
- Knee leads hip_pitch by +30° (foot clearance before weight transfers)
- Hip roll leads hip_pitch by +120° (CoM shifts before swing leg lifts)
- Ankle roll = −hip_roll (always)

Amplitudes: hip_pitch=31°, knee=31°, hip_roll=11°, ankle_roll=11°. Period=1.2s.
DC knee bias `CPG_KNEE_STANDING_OFFSET_DEG=23°` keeps legs crouched at stance.

## IK Walk (ENABLE_WALK)

4-phase state machine: SHIFT_RIGHT → SWING_LEFT → SHIFT_LEFT → SWING_RIGHT.
- SHIFT: lerp CoM over stance foot via hip roll; advances when IMU roll > `WALK_SHIFT_THRESHOLD_DEG=8°` OR timeout.
- SWING: sinusoidal float leg + stance knee lowers for propulsion. Purely timed.
- IMU balance corrections applied on top of walk output every cycle.

## Kinematics Reference

FK sagittal: `x = L1·cos(θ2) + L2·cos(θ2+θ3) + L3`, `y = L1·sin(θ2) + L2·sin(θ2+θ3)`
FK coronal: `z = (L1+L2)·sin(θ1)` (ankle constraint simplifies this)

IK: Step 1 θ1=arcsin(z/(L1+L2)) → Step 2 reach d=√((x−L3)²+y²) → Step 3 knee (law of cosines) → Step 4 hip_pitch → Step 5 θ4=−θ1.

`IKElbow::UP` or `::DOWN` selects knee configuration. Returns `valid=false` if out of workspace.

Practical workspace for stable gait: x ±50mm, y −120 to −170mm, z ±20mm.

## IMU & Stabilizer

Complementary filter: `θ_est = 0.98·(θ_prev + ω·Δt) + 0.02·θ_accel`

IMU axes (physical orientation):
- X: right shoulder → left shoulder (lateral)
- Y: feet → head (up when standing)
- Z: rear → front (sagittal)

`IMU_PITCH_SIGN = −1` (inverted on real hardware), `IMU_ROLL_SIGN = +1`.

Safety cutoff: if tilt > 50° the active experiment calls `servos.set_all_neutral()` and suspends. Board reset required to resume.

## FreeRTOS Tasks (single-core, all pinned to core 0)

| Task | Hz | Priority |
|---|---|---|
| `task_imu` | 200 | 3 (highest) |
| active experiment | 100 | 2 |
| `task_telemetry` | 50 (broadcasts at 10) | 1 |
| `task_serial_console` | — | 0 (SLB mode only) |

Shared globals protected by `state_mutex`: `g_imu_estimate`, `g_leg_angles`.

## Telemetry WebSocket

Connect to `ws://<robot-ip>:81`. Robot joins WiFi defined in `config.h` (`WIFI_SSID`, `WIFI_PASSWORD`).

State broadcast (10 Hz): JSON with `t`, `pitch`, `roll`, `left`/`right` joint angles, `cpg` params.

Incoming commands: `set_period`, `set_amp_hp/hr/k/ar`, `set_sway_target`, `set_neutral`, `calibrate_imu`.

## Tools

- `tools/joycon_sway.py` — **deprecated**. Was used to control IK sway via JoyCon over WiFi/HTTP. Too slow. Will be replaced with a BLE library connecting JoyCon directly to the robot.

## Docs

- [docs/JOINTS.md](docs/JOINTS.md) — full hardware reference, PCA9685 channel map, servo direction table
- [docs/kinematics.md](docs/kinematics.md) — FK/IK derivations, DH parameters, workspace analysis, experiment findings
- [docs/imu-balance.md](docs/imu-balance.md) — IMU axis orientation, complementary filter, balance controller gains
- [docs/logs/](docs/logs/) — dev logs with experiment results and tuning history

## Simulation

`simulation/sim.py` — early MuJoCo prototype for Optimus. See `/mujuco/` for the main simulation environment (covers both Optimus and Spot Micro).
