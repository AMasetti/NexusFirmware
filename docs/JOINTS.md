# Optimus Biped — Joints & Firmware Reference

## Hardware

| Component | Part | Interface |
|-----------|------|-----------|
| MCU | ESP32-C3 Super Mini | USB-CDC (native, no UART chip) |
| Servo driver | PCA9685 16-ch PWM | I2C @ 0x40, 400 kHz |
| IMU | MPU6050 | I2C @ 0x68, 400 kHz |
| Leg servos | MG995 (×8) | PWM 600–2650 µs, ±90° |
| Arm servos | Futaba S3003 (×6) + hip yaw (×1) | PWM 900–2100 µs, ±90° |
| I2C pins | SDA = GPIO 8, SCL = GPIO 9 | 400 kHz |

---

## Leg geometry

```
Hip joint ──── femur (L1 = 90 mm) ──── Knee ──── tibia (L2 = 90 mm) ──── Ankle ──── foot (L3 = 30 mm)
```

Physical reference points:
- Foot pad: 125 mm long × 7 mm wide
- Ankle joint: 40 mm above ground
- CoM (at hip joint): ~250 mm above ground when standing upright

Coordinate frame (per leg, origin at hip):

| Axis | Direction |
|------|-----------|
| X | Forward |
| Y | Up |
| Z | Lateral (outward) |

---

## Leg joint definitions

Each leg has **4 DOF**. Angles are computed internally in radians; all configuration parameters and docs use degrees.

| # | Name | Axis | Motion | Range |
|---|------|------|--------|-------|
| θ1 | Hip roll | Z (lateral) | Lateral weight shift | ±90° |
| θ2 | Hip pitch | X (sagittal) | Forward/backward swing | ±90° |
| θ3 | Knee | X (sagittal) | Knee bend | ±90° |
| θ4 | Ankle roll | Z (lateral) | Foot leveling | ±90° |

> **Ankle constraint**: θ4 = −θ1 always. The ankle compensates hip roll to keep the foot flat on the ground.

---

## PCA9685 channel mapping

### Leg channels — MG995 servos (PWM 600–2650 µs)

| PCA9685 ch | Leg | CPG joint | Old name | set_inverse | Halt angle |
|-----------|-----|-----------|----------|-------------|-----------|
| 0 | Right | ankle_roll (θ4) | RightLeg.Ankle | False | 90° |
| 1 | Right | knee (θ3) | RightLeg.Knee_Bottom | False | 80° |
| 2 | Right | hip_pitch (θ2) | RightLeg.Knee_Top | True | 100° |
| 3 | Right | hip_roll (θ1) | RightLeg.Hip | True | 100° |
| 12 | Left | hip_roll (θ1) | LeftLeg.Hip | False | 80° |
| 13 | Left | hip_pitch (θ2) | LeftLeg.Knee_Top | False | 90° |
| 14 | Left | knee (θ3) | LeftLeg.Knee_Bottom | True | 90° |
| 15 | Left | ankle_roll (θ4) | LeftLeg.Ankle | True | 70° |

### Upper-body channels — Futaba S3003 servos (PWM 900–2100 µs)

T-pose = 90° on all arm channels (pulse center, no trim needed).

| PCA9685 ch | Joint | T-pose |
|-----------|-------|--------|
| 4 | R shoulder forward/backward | 90° |
| 5 | R shoulder lateral elevation | 90° |
| 6 | R forearm lateral elevation | 90° |
| 7 | L shoulder forward/backward | 90° |
| 8 | L shoulder lateral elevation | 90° |
| 9 | L forearm lateral elevation | 90° |
| 10 | Hip yaw (waist rotation) | 90° |
| 11 | — unused — | — |

### Servo angle conventions — raw physical direction (before set_inverse)

All descriptions below are from the **raw physical servo perspective** — what the robot does when the servo angle increases or decreases, **before** any `set_inverse` / `SERVO_DIR` correction is applied. This is the hardware ground truth: what you observe if you command a servo directly.

Servos with `set_inverse=True` will have their positive direction **flipped** relative to what the CPG/code sends — the firmware applies `× SERVO_DIR` to normalize this for you.

#### Right leg

| Joint | set_inverse | Raw (+) | Raw (−) |
|-------|-------------|---------|---------|
| `hip_roll` | **True** | body tilts RIGHT — CoM shifts over right foot | body tilts LEFT — CoM away from right foot |
| `hip_pitch` | **True** | right leg swings FORWARD | right leg swings BACKWARD |
| `knee` | False | knee EXTENDS — leg crouches | knee BENDS  — leg straightens little bit |
| `ankle_roll` | False | right foot rolls OUTWARD — inside edge down | right foot rolls INWARD — outside edge down |

#### Left leg

| Joint | set_inverse | Raw (+) | Raw (−) |
|-------|-------------|---------|---------|
| `hip_roll` | False | body tilts RIGHT — CoM shifts over right foot | body tilts LEFT — CoM away from right foot |
| `hip_pitch` | False | left leg swings BACKWARD | left leg swings FORWARD |
| `knee` | **True** | knee EXTENDS — leg straightens | knee BENDS — leg crouches |
| `ankle_roll` | **True** | left foot rolls INWARD — outside edge down | left foot rolls OUTWARD — inside edge down |

> **Hip roll**: both servos raw (+) push the CoM in the same direction (right). The `set_inverse=True` on the right hip roll corrects for its mirrored physical mounting — so the CPG can use the same sign convention (positive = shift CoM toward that leg) across both legs after correction.

> **Ankle constraint**: in IK, ankle_roll = −hip_roll. The opposing `set_inverse` flags on each leg's ankle vs. hip are what make this single equation work correctly for both legs.

#### Upper body (all set_inverse=False)

| Joint | Raw (+) | Raw (−) |
|-------|---------|---------|
| `shoulder_FB` (R & L) | arm swings FORWARD | arm swings BACKWARD |
| `shoulder_LAT` (R & L) | arm raises — away from T-pose toward overhead | arm lowers — drops below T-pose |
| `forearm_LAT` (R & L) | forearm raises | forearm lowers |
| `hip_yaw` | verify direction on hardware | opposite rotation |

> **Arm mirroring in code**: R shoulder uses `+deg`, L shoulder uses `−deg` for the same logical motion (both arms forward together). Applied in `exp_walk` and `exp_slb` — do not re-mirror when setting arm angles in experiments.

---

### Servo direction and offset — leg channels

Each servo has a **direction** (±1) and a **DC offset** applied before writing to the PCA9685:

```
physical_angle_deg = joint_rad × (180/π) × direction + offset_deg
```

| ch | Direction | Offset (°) | Note |
|----|-----------|-----------|------|
| 0 | +1 | +8 | R ankle_roll — matches hip roll offset, flat foot at halt |
| 1 | +1 | −10 | R knee — halt 80° |
| 2 | −1 | +10 | R hip_pitch — set_inverse=True, halt 100° |
| 3 | −1 | +8 | R hip_roll — wider stance, prevents foot collision |
| 12 | +1 | −8 | L hip_roll — wider stance, prevents foot collision |
| 13 | +1 | 0 | L hip_pitch — halt 90° |
| 14 | −1 | 0 | L knee — halt 90° |
| 15 | −1 | −8 | L ankle_roll — matches hip roll offset, flat foot at halt |

> To tune: adjust `SERVO_OFFSET_DEG_*` in `include/config.h` after confirming mechanical directions on the bench.

---

## CPG gait algorithm

The walking pattern is generated by a **Central Pattern Generator (CPG)** — each joint oscillates as a sinusoid with fixed phase relationships.

### Oscillator equation

```
θᵢ(t) = Aᵢ · sin(ω·t + φᵢ) + θᵢ_offset
```

Where `ω = 2π / T` and `T = CPG_PERIOD_S` (default 1.2 s).

### Current amplitudes

| Joint | Amplitude |
|-------|-----------|
| Hip pitch | 31° |
| Knee | 31° |
| Hip roll | 11° |
| Ankle roll | 11° |

### Phase relationships

| Joint | Left phase | Right phase | Offset from hip pitch |
|-------|-----------|-------------|----------------------|
| Hip pitch | 0° (base) | 180° | — anti-phase |
| Knee | 0° + 30° | 180° + 30° | +30° ahead of hip pitch |
| Hip roll | 0° + 120° | 180° + 120° | +120° ahead of hip pitch (CoM shifts before swing) |
| Ankle roll | −hip_roll | −hip_roll | mirrors hip roll to keep foot level |

### Standing knee offset

A DC bias of **23°** is added to both knee joints so legs stay crouched at stance. The CPG oscillates around this offset.
Increase `CPG_KNEE_STANDING_OFFSET_DEG` in `config.h` if the robot collapses under its own weight.

### Per-leg DC trims

| Parameter | Default | Effect |
|-----------|---------|--------|
| `CPG_HIP_ROLL_TRIM_L / _R` | 0° | DC lateral bias per leg (centered on halt offsets) |
| `CPG_HIP_PITCH_TRIM_L_DEG` | 0° | Stride length trim, left leg |
| `CPG_HIP_PITCH_TRIM_R_DEG` | −10° | Cancels the +10° servo offset bias on right hip pitch |

---

## Forward kinematics (FK)

Computes foot position `(x, y, z)` in mm given joint angles.

**Sagittal plane** (θ2, θ3):
```
x = L1·cos(θ2) + L2·cos(θ2 + θ3) + L3
y = L1·sin(θ2) + L2·sin(θ2 + θ3)
```

**Coronal plane** (θ1, θ4):
```
z = (L1 + L2)·sin(θ1) + L3·sin(θ1 + θ4)
```

With ankle constraint θ4 = −θ1 → coronal simplifies to `z = (L1 + L2)·sin(θ1)`.

Two implementations exist for cross-validation:
- `fk_solve()` — analytical per-plane decomposition; used in the real-time loop
- `fk_full_dh()` — full 4×4 DH chain; used for verification

---

## Inverse kinematics (IK)

Solves joint angles for a desired foot position `(x, y, z)` in mm. Elbow configuration selectable: `IKElbow::UP` or `IKElbow::DOWN`.

**Step 1** — Hip roll from lateral target (ankle constraint applied):
```
θ1 = arcsin(z / (L1 + L2))
```

**Step 2** — Effective sagittal reach (subtract rigid foot):
```
d = √((x − L3)² + y²)
```

**Step 3** — Knee by law of cosines:
```
cos(θ3) = (d² − L1² − L2²) / (2·L1·L2)
sin(θ3) = ±√(1 − cos²(θ3))    [sign = +UP, −DOWN]
θ3 = atan2(sin(θ3), cos(θ3))
```

**Step 4** — Hip pitch:
```
θ2 = atan2(y, x−L3) − atan2(L2·sin(θ3), L1 + L2·cos(θ3))
```

**Step 5** — Ankle compensation:
```
θ4 = −θ1
```

All angles clamped to `SERVO_MIN_DEG` / `SERVO_MAX_DEG` (±90°) after solving.
Returns `valid = false` if target is outside reachable workspace.

---

## IMU stabilizer

Requires `USE_IMU 1` in `config.h` and a physical MPU6050 connected.

**Complementary filter**:
```
θ_est = α·(θ_prev + ω·Δt) + (1−α)·θ_accel      α = 0.98
```

**IMU physical orientation**:
- X axis: right shoulder → left shoulder (lateral)
- Y axis: feet → head (vertical, up when standing)
- Z axis: rear → front (sagittal, forward)

**Sign convention** (tuned to physical hardware):
- `IMU_PITCH_SIGN = −1` — pitch correction is inverted
- `IMU_ROLL_SIGN = +1` — positive when leaning right

**Stabilizer dead-bands and corrections**:

| Disturbance | Correction applied | Threshold |
|-------------|-------------------|-----------|
| Forward pitch | Hip pitch offset on both legs | 0.6° |
| Lateral roll | Ankle roll offsets (left/right) | 0.6° |

### Safety cutoff

If tilt exceeds **50°** on any axis, the active experiment task calls `servos.set_all_neutral()` and suspends itself. A board reset is required to resume.

---

## FreeRTOS task layout

ESP32-C3 is **single-core** — all tasks pin to core 0.

| Task | Frequency | Stack | Priority | Active when |
|------|-----------|-------|----------|-------------|
| `task_imu` | 200 Hz | 4 KB | 3 (highest) | `USE_IMU=1` |
| active experiment | 100 Hz | 8 KB | 2 | matching `ENABLE_*` flag |
| `task_telemetry` | 10 Hz | 8 KB | 1 | always |
| `task_serial_console` | — | 3 KB | 0 | `ENABLE_SINGLE_LEG_BALANCE=1` only |

A single `state_mutex` protects shared `g_imu_estimate` and `g_leg_angles`.

---

## Telemetry WebSocket

Connect to `ws://<robot-ip>:81` after the robot joins the network defined in `config.h`.

### Outgoing state (every 100 ms)

```json
{
  "t": 12345,
  "pitch": 0.012,
  "roll": -0.003,
  "left":  { "hr": 0.05, "hp": 0.21, "k": 0.38, "ar": -0.05 },
  "right": { "hr": -0.05, "hp": -0.21, "k": 0.38, "ar": 0.05 },
  "cpg":  { "period": 1.0, "amp_hp": 0.35, "amp_hr": 0.15, "amp_k": 0.25 }
}
```

### Incoming commands

| Command | Effect |
|---------|--------|
| `{"cmd":"set_period","value":1.2}` | Change gait cycle period (s) |
| `{"cmd":"set_amp_hp","value":0.4}` | Change hip pitch amplitude (rad) |
| `{"cmd":"set_amp_hr","value":0.2}` | Change hip roll amplitude (rad) |
| `{"cmd":"set_amp_k","value":0.3}` | Change knee amplitude (rad) |
| `{"cmd":"set_amp_ar","value":0.1}` | Change ankle roll amplitude (rad) |
| `{"cmd":"set_neutral"}` | Move all servos to 0° |
| `{"cmd":"calibrate_imu"}` | Re-run gyro bias calibration |

---

## Build & flash

```bash
cd optimus/firmware

make setup          # first time: installs ESP32 core + WebSockets library
make compile        # build
make compile-upload # build + flash
make monitor        # serial output at 115200 baud
```

---

## `config.h` mode flags

Exactly one `ENABLE_*` flag active at a time. Compile-time `static_assert` guards enforce mutual exclusivity.

| Flag | Default | Description |
|------|---------|-------------|
| `ENABLE_GAIT` | 0 | Open-loop CPG walking gait |
| `ENABLE_WALK` | 0 | IMU-gated 4-phase walk state machine |
| `ENABLE_BALANCE` | 0 | Static two-foot IMU balance |
| `ENABLE_SINGLE_LEG_BALANCE` | 1 | Balance on right foot, left leg raised |
| `USE_IMU` | 1 | Enable MPU6050; required by WALK, BALANCE, SLB |
| `SERVO_SWEEP_TEST` | 0 | Sweep each leg channel ±20° on boot (bench test) |

## `config.h` key tuning parameters

| Constant | Value | Description |
|----------|-------|-------------|
| `L1_MM` | 90 mm | Femur length |
| `L2_MM` | 90 mm | Tibia length |
| `L3_MM` | 30 mm | Foot length |
| `CPG_PERIOD_S` | 1.2 s | Gait cycle period |
| `CPG_HIP_PITCH_AMP_DEG` | 31° | Hip pitch oscillation amplitude |
| `CPG_KNEE_AMP_DEG` | 31° | Knee oscillation amplitude |
| `CPG_HIP_ROLL_AMP_DEG` | 11° | Hip roll oscillation amplitude |
| `CPG_ANKLE_ROLL_AMP_DEG` | 11° | Ankle roll amplitude |
| `CPG_KNEE_STANDING_OFFSET_DEG` | 23° | DC knee bend at stance |
| `SAFETY_TILT_LIMIT_DEG` | 50° | Tilt angle that triggers servo cutoff |
| `COMPLEMENTARY_ALPHA` | 0.98 | IMU complementary filter coefficient |
| `STAB_PITCH_THRESHOLD_DEG` | 0.6° | Pitch dead-band for stabilizer |
| `STAB_ROLL_THRESHOLD_DEG` | 0.6° | Roll dead-band for stabilizer |
| `SERVO_DIR_*` | ±1.0 | Per-servo mechanical direction |
| `SERVO_OFFSET_DEG_*` | (°) | Per-servo DC physical offset |
