# Optimus Biped — Kinematics Reference

## Joint Configuration (8 DOF, 4 per leg)

| Joint | Variable | Plane | Servo |
|-------|----------|-------|-------|
| Hip Roll (lateral abduction) | θ₁ | Coronal  | MG995 |
| Hip Pitch (flex/extension)   | θ₂ | Sagittal | MG995 |
| Knee Pitch                   | θ₃ | Sagittal | MG995 |
| Ankle Roll (lateral ankle)   | θ₄ | Coronal  | MG995 |

**Plane decoupling insight:** θ₁/θ₄ operate in the coronal plane (lateral stability) while θ₂/θ₃ operate in the sagittal plane (propulsion). This decoupling allows FK and IK to be solved independently per plane — greatly simplifying the controller.

## Link Lengths

| Symbol | Description | Value |
|--------|-------------|-------|
| L1 | Femur (hip → knee)  | ~100 mm |
| L2 | Tibia (knee → ankle) | ~100 mm |
| L3 | Foot (rigid, horizontal) | ~50 mm |

---

## Forward Kinematics

### Sagittal Plane (θ₂, θ₃) — pure 2R chain

```
x = L1·cos(θ₂) + L2·cos(θ₂+θ₃) + L3
y = L1·sin(θ₂) + L2·sin(θ₂+θ₃)
```

L3 is added as a constant offset in x because the foot is rigid and horizontal in the sagittal plane. The ankle joint does **not** act in the sagittal direction.

### Coronal Plane (θ₁, θ₄)

```
z = (L1+L2)·sin(θ₁) + L3·sin(θ₁+θ₄)
```

### Full Homogeneous Transform (DH Convention)

```
T_foot = T1(θ₁) · T2(θ₂) · T3(θ₃) · T4(θ₄)
```

Each `Ti` is a 4×4 Denavit-Hartenberg matrix:

```
      ┌ cθ   -sθ·cα   sθ·sα   a·cθ ┐
Ti =  │ sθ    cθ·cα  -cθ·sα   a·sθ │
      │  0       sα      cα      d  │
      └  0        0       0      1  ┘
```

DH parameter table:

| i | θ  | d | a  | α    | Joint |
|---|----|----|----|----- |-------|
| 1 | θ₁ | 0 | 0  | π/2  | Hip Roll |
| 2 | θ₂ | 0 | L1 | 0    | Hip Pitch |
| 3 | θ₃ | 0 | L2 | 0    | Knee |
| 4 | θ₄ | 0 | L3 | 0    | Ankle Roll |

---

## Inverse Kinematics

Given desired foot position **[x, y, z]** → find **[θ₁, θ₂, θ₃, θ₄]**.

### Step 1 — Decouple hip roll (coronal)

```
θ₁ = atan2(z, y_lateral)
```

where `y_lateral = -(L1+L2)` is the nominal leg height in stance.

### Step 2 — Effective 2D reach (subtract rigid foot)

```
d = √((x - L3)² + y²)
```

### Step 3 — Knee by law of cosines

```
cos(θ₃) = (d² - L1² - L2²) / (2·L1·L2)
θ₃ = atan2(±√(1 - cos²(θ₃)), cos(θ₃))
```

- `+` sign → elbow-up (knee forward)
- `−` sign → elbow-down (knee backward)

**Singularity:** if `|cos(θ₃)| > 1`, the target is out of reach.

### Step 4 — Hip pitch

```
θ₂ = atan2(y, x-L3) - atan2(L2·sin(θ₃), L1 + L2·cos(θ₃))
```

### Step 5 — Ankle roll (critical!)

```
θ₄ = −θ₁
```

This cancels the hip roll rotation so the foot remains parallel to the ground in the coronal plane. **If this relation fails, the robot falls laterally.**

---

## CPG Gait Equations

Each joint oscillates as:

```
θᵢ(t) = Aᵢ · sin(ω·t + φᵢ) + θᵢ_offset
```

where `ω = 2π/T`, `T ≈ 1.0 s`.

### Phase relationships (forward gait)

| Joint pair | Phase offset |
|------------|-------------|
| Hip pitch L vs R | π (anti-phase) |
| Knee vs hip pitch (same leg) | −π/4 |
| Hip roll vs hip pitch (same leg) | π/2 |
| Ankle roll | = −hip_roll (always) |

### Gait phases (full cycle)

| Phase | Left leg | Right leg |
|-------|----------|-----------|
| t ∈ [0, T/4)   | Single support | Swing |
| t ∈ [T/4, T/2) | Double support (transition) | Landing |
| t ∈ [T/2, 3T/4)| Swing | Single support |
| t ∈ [3T/4, T)  | Landing | Double support (transition) |

---

## Complementary Filter (IMU Fusion)

```
θ_est = α·(θ_prev + ω_gyro·Δt) + (1−α)·θ_accel
α = 0.98
```

The gyroscope dominates at high frequency (avoids accel noise). The accelerometer corrects gyro drift at low frequency.

Accel angles:
```
pitch_accel = atan2(−ax, √(ay² + az²))
roll_accel  = atan2(ay, az)
```

---

## Workspace Analysis

Maximum reach (sagittal): `r_max = L1 + L2 = 200 mm`
Minimum reach: `r_min ≈ |L1 − L2| = 0 mm` (singular: leg fully folded)

Practical operating range for stable gait:
- Forward/backward (x): ±50 mm relative to neutral stance
- Height (y): −120 mm to −170 mm (above ground)
- Lateral (z): ±20 mm

---

## Servo Direction Convention and Joint-Space Angles

`JointAngles` values passed to `write_leg()` are **not** the same as physical servo angles.
The mapping is:

```
servo_deg = joint_rad × RAD2DEG × SERVO_DIR[ch] + SERVO_OFFSET_DEG[ch]
```

Key implications:

| Joint | ch | dir | offset | Effect |
|-------|----|-----|--------|--------|
| R hip_pitch | 2 | −1 | +10° | Positive joint angle → servo moves negative (backward) |
| R knee | 1 | +1 | −10° | Positive joint angle → servo moves positive (bent) |
| L hip_pitch | 13 | +1 | 0° | Positive joint angle → servo moves positive (forward) |
| L knee | 14 | −1 | 0° | Positive joint angle → servo moves negative (bent) |

**Halt pose** (`set_all_neutral()`): all servos at 0° (center pulse). Legs fully straight, T-pose arms.

**Joint-space zero** (`JointAngles = {0,0,0,0}`): equivalent to the halt pose with servo offsets applied — legs straight with the small per-channel trim offsets baked in. This is the FK reference origin.

**FK angle relationship:**
```
θ₂_fk = hip_pitch_joint − π/2
```
Where `θ₂_fk` is the absolute femur angle from horizontal used in the FK/IK equations, and `hip_pitch_joint = 0` corresponds to the femur pointing straight down (−π/2 from horizontal).

---

## Reference Poses

### Halt / T-pose
- All joint angles = 0
- Legs fully straight, femur vertical
- Foot position relative to hip: x = +30 mm (L3 forward), y = −180 mm (L1+L2 below)

### Symmetric Squat (IK sway neutral)
- Target: equal **visual** bend at hip and knee
- Because hip_pitch has `dir = −1` and knee has `dir = +1`, equal servo travel requires equal joint angles:
  - `hip_pitch_joint = IK_SWAY_KNEE_DEG = 45°`
  - `knee_joint = IK_SWAY_KNEE_DEG = 45°`
- FK angles: `θ₂_fk = 45° − 90° = −45°` (femur 45° forward of vertical), `θ₃_fk = 45°`
- Foot position relative to hip:
  - x = L1·cos(−45°) + L2·cos(0°) + L3 = 63.6 + 90 + 30 = **183.6 mm** forward
  - y = L1·sin(−45°) + L2·sin(0°) = **−63.6 mm** below hip
- Hip height above ankle: ~63.6 mm — deep squat position
- **Verified on hardware:** robot stands in equilibrium at this pose with no active balance correction needed.

---

## IK Sagittal Sway — Experiment Findings

### Setup
- Both feet fixed on ground, hips oscillate ±`IK_SWAY_AMP_MM` (20 mm) forward/backward in the sagittal plane.
- Period: `IK_SWAY_PERIOD_S` = 4.0 s. Motion profile: **cosine easing** (`δ = amp·(−cos(ωt))`).
- IK recomputes `hip_pitch` and `knee` every 100 Hz cycle to keep foot contact point fixed.

### Cosine easing
Using `−cos(ωt)` instead of `sin(ωt)` gives zero velocity at both extremes (forward and backward stops). The robot glides to each limit and reverses smoothly. `sin()` caused abrupt snapping at the peaks.

### Workspace boundary behaviour
At the extremes of the sway range the IK reaches the boundary of the reachable workspace (`|cos_k| > 1`). Clamping `cos_k` to [−1, 1] instead of falling back to neutral eliminates the vibration that occurred when the solver snapped between the last valid solution and the fallback pose.

### Observed kinematics
- Forward extreme: upper leg (femur) clutches more, lower leg (tibia) extends — hip_pitch increases, knee decreases.
- Backward extreme: upper leg extends, lower leg clutches more — hip_pitch decreases, knee increases.
- Hip height (y) stays constant throughout; only the sagittal foot-to-hip vector rotates.

### Key assumptions validated on hardware
1. `hip_pitch_joint = 0` → femur points straight down (θ₂_fk = −π/2). Confirmed: halt pose has fully straight legs.
2. `dir = −1` on R hip_pitch inverts servo travel — a positive joint angle moves the servo backward. To achieve equal visual bend with the knee (`dir = +1`), both joints need the same numeric angle (45°), not half.
3. FK origin (foot_x0, foot_y0) must be derived from FK at the actual neutral joint angles, not assumed to be (0, 0). Incorrect origin caused the robot to start already tilted and fall forward.
4. `IK_SWAY_AMP_MM = 20 mm` stays within the reachable workspace at the 45° squat depth. Larger amplitudes will saturate at the boundary.
