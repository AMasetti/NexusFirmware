# IMU Balance Controller

## Hardware: MPU6050 / MPU6501

The IMU is mounted on the robot with the following physical axis orientation:

| Chip axis | Physical direction           |
|-----------|------------------------------|
| **X**     | Right shoulder → Left shoulder (lateral) |
| **Y**     | Feet → Head (vertical / up when standing) |
| **Z**     | Rear → Front (sagittal, by right-hand rule X×Y) |

When the robot is standing upright, gravity acts in the **−Y** direction.  
The accelerometer reads `ay ≈ +1 g`, `ax ≈ 0`, `az ≈ 0`.

---

## Tilt Estimation

### Body pitch (forward / backward lean)
Rotation around the **X** axis (the lateral axis across the shoulders).

| Signal | Derivation |
|--------|-----------|
| Accel estimate | `atan2(−az, ay)` — when the head tips forward, az gains a negative gravity component |
| Gyro rate      | `gyro_x` |
| Complementary filter | `pitch = α·(pitch + gyro_x·dt) + (1−α)·pitch_accel` |

Sign convention: **positive pitch = leaning forward**.

### Body roll (lateral lean)
Rotation around the **Z** axis (the forward axis).

| Signal | Derivation |
|--------|-----------|
| Accel estimate | `atan2(ax, ay)` — when the robot leans right the left side rises, pushing gravity into the +X direction |
| Gyro rate      | `gyro_z` |
| Complementary filter | `roll = α·(roll + gyro_z·dt) + (1−α)·roll_accel` |

Sign convention: **positive roll = leaning right**.

### Body yaw rate
Rotation around the **Y** axis (the vertical axis).  
No absolute heading can be recovered from the accelerometer, so only the gyro rate is used.

```
yaw_rate = gyro_y   [rad/s]
```

Positive yaw rate = rotating counter-clockwise when viewed from above.

---

## Complementary Filter Parameters

| Define | Default | Purpose |
|--------|---------|---------|
| `COMPLEMENTARY_ALPHA` | 0.98 | Weight of gyro vs accel (0 = accel only, 1 = gyro only) |
| `IMU_PITCH_SIGN` | +1 | Flip if corrections go the wrong way on real hardware |
| `IMU_ROLL_SIGN`  | +1 | Flip if corrections go the wrong way on real hardware |

---

## Balance Controller (`ENABLE_BALANCE=1`)

Active balance is a separate firmware mode (requires `USE_IMU=1`, `ENABLE_GAIT=0`).  
A FreeRTOS task runs at **100 Hz** and applies three independent proportional corrections:

### 1. Pitch correction → hip pitch (both legs equally)
```
correction = −BALANCE_PITCH_GAIN × pitch_rad
left.hip_pitch  += correction
right.hip_pitch += correction
```
Applied only when `|pitch| > STAB_PITCH_THRESHOLD_RAD`.

### 2. Roll correction → hip roll (symmetric, opposite signs)
```
correction = −BALANCE_ROLL_GAIN × roll_rad
left.hip_roll  += +correction
right.hip_roll += −correction
```
Applied only when `|roll| > STAB_ROLL_THRESHOLD_RAD`.

### 3. Yaw damping → hip yaw servo (ch 10, Futaba S3003)
```
yaw_correction_deg = −BALANCE_YAW_RATE_GAIN × yaw_rate_rad_s × (180/π)
yaw_correction_deg = clamp(yaw_correction_deg, ±BALANCE_YAW_MAX_DEG)
pulse_us = 1500 + yaw_correction_deg / 90 × 600
```
Center = 1500 µs (facing forward). Range: 900–2100 µs.  
This keeps the upper body (torso + arms) in the same azimuthal plane despite leg dynamics or external perturbations.

### Gains

| Define | Default | Effect |
|--------|---------|--------|
| `BALANCE_PITCH_GAIN`    | 0.6 | Aggressiveness of forward/backward correction |
| `BALANCE_ROLL_GAIN`     | 0.6 | Aggressiveness of lateral correction |
| `BALANCE_YAW_RATE_GAIN` | 0.3 | Damping of rotational drift |
| `BALANCE_YAW_MAX_DEG`   | 30° | Hard clamp on hip yaw travel |

### Safety cutoff
If `|pitch| > SAFETY_TILT_LIMIT_RAD` **or** `|roll| > SAFETY_TILT_LIMIT_RAD`, all servo outputs are cut and the balance task suspends itself. Reset the ESP32 to resume.

---

## Firmware Mode Flags (config.h)

| `ENABLE_GAIT` | `ENABLE_BALANCE` | Behaviour |
|:---:|:---:|---|
| 0 | 0 | Passive hold — legs at halt pose, arms at T-pose. No IMU needed. |
| 0 | 1 | Active balance — IMU corrections keep body upright. Requires `USE_IMU=1`. |
| 1 | 0 | CPG walking gait. IMU stabilisation active only if `USE_IMU=1`. |

> `ENABLE_BALANCE=1` with `ENABLE_GAIT=1` is a **compile-time error** (`static_assert`).

---

## Tuning Notes

1. **First run**: set all gains to 0, verify the IMU reads sensible angles on Serial.
2. **Axis signs**: if corrections oppose the lean instead of correcting it, flip `IMU_PITCH_SIGN` or `IMU_ROLL_SIGN`.
3. **Pitch gain**: increase `BALANCE_PITCH_GAIN` in steps of 0.1 until oscillation starts, then back off 20%.
4. **Roll gain**: same procedure as pitch.
5. **Yaw gain**: the hip yaw servo has limited authority — if it saturates at `BALANCE_YAW_MAX_DEG`, reduce `BALANCE_YAW_RATE_GAIN`.
