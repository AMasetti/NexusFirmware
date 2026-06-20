#pragma once

// ─── Robot geometry ───────────────────────────────────────────────────────────
#define L1_MM     90.0f   // femur  (hip → knee)   [mm]
#define L2_MM     90.0f   // tibia  (knee → ankle)  [mm]
#define L3_MM     30.0f   // foot   (ankle → toe)   [mm]
// Physical foot pad: 125 mm long × 7 mm wide.  Ankle joint 40 mm above ground.
// CoM at hip joint: ~250 mm above ground when standing upright.

// ─── Angle unit convention ────────────────────────────────────────────────────
// All angle parameters in this file are in degrees [°].
// Multiply by DEG2RAD (defined per translation unit) before use in radians.

// ─── Servo limits — MG995 (legs) ──────────────────────────────────────────────
// Pulse range matched to old robot (servos_lib.py MIN_IMP=600, MAX_IMP=2650).
// Center (0°) → (600+2650)/2 = 1625 µs, which equals halt=90° in the old system.
#define SERVO_MIN_DEG   -90.0f
#define SERVO_MAX_DEG    90.0f
#define SERVO_PWM_MIN    600     // µs  (was 500 — matched to old robot)
#define SERVO_PWM_MAX   2650     // µs  (was 2400 — matched to old robot)

// ─── Servo limits — Futaba S3003 (arms) ───────────────────────────────────────
// Standard Futaba S3003: 900–2100 µs, center at 1500 µs (90°).
// T-pose = 90° on all arm channels = pulse center, zero trim offset needed.
#define ARM_SERVO_PWM_MIN    900   // µs
#define ARM_SERVO_PWM_MAX   2100   // µs

// ─── I2C ──────────────────────────────────────────────────────────────────────
// ESP32-C3 only has GPIO 0-21 (GPIO 22 does not exist).
// Super Mini C3 exposes: 0-10, 20, 21.
// Change these to match your actual wiring.
#define I2C_SDA_PIN      8
#define I2C_SCL_PIN      9
#define I2C_FREQ_HZ     400000

// ─── Debug flags ─────────────────────────────────────────────────────────────
// Set to 1 to run the servo sweep test on boot (verifies each channel moves).
// Set to 0 for normal operation.
#define SERVO_SWEEP_TEST  0

// ─── Gait enable ─────────────────────────────────────────────────────────────
// 0 = hold halt pose (legs) + T-pose (arms) indefinitely — use for servo centering.
// 1 = run CPG walking gait normally.
#define ENABLE_GAIT  0

// ─── IMU-gated walk enable ────────────────────────────────────────────────────
// Requires USE_IMU=1. Mutually exclusive with ENABLE_GAIT and ENABLE_BALANCE.
// 4-phase state machine: SHIFT_RIGHT → SWING_LEFT → SHIFT_LEFT → SWING_RIGHT
//   SHIFT phases: lerp CoM over stance foot; advance when IMU confirms roll OR timeout
//   SWING phases: sinusoidal float leg + stance knee lower; purely timed
#define ENABLE_WALK  0

// Walk tuning parameters
#define WALK_SHIFT_ROLL_DEG        14.0f  // hip roll amplitude for weight shift [°] — needs margin above geometric minimum (~6.6°)
#define WALK_SHIFT_THRESHOLD_DEG    8.0f  // IMU roll confirming CoM over stance foot [°]
#define WALK_SHIFT_DURATION_S      0.70f  // max duration of each shift phase [s]
#define WALK_SWING_DURATION_S      0.50f  // duration of each swing phase [s]
#define WALK_SWING_HP_DEG          49.0f  // float leg hip_pitch swing amplitude [°] — 90mm femur → 68mm step
#define WALK_SWING_KNEE_DEG        20.0f  // float leg knee flex [°] — bends knee to lift foot off ground
#define WALK_STANCE_LOWER_DEG      46.0f  // stance leg knee bend for propulsion [°] — drops CoM ~27mm per step
#define WALK_ARM_SWING_DEG         10.0f  // shoulder FB swing amplitude during gait — keep small, large swing creates yaw couple
// Arm resting pose during walk — same pulse sent to both sides (NOT mirrored)
// so both arms hang symmetrically. Positive = smaller pulse = arms down on both.
#define WALK_ARM_LAT_REST_DEG     105.0f   // shoulder lateral: ~65° below horizontal (~30% lower)
#define WALK_ARM_FOREARM_REST_DEG 45.0f   // forearm lateral:  ~45° droop (~30% lower)

// SHIFT phase stability gate: must hold IMU condition for this many consecutive 100 Hz
// cycles before advancing to swing. Prevents rushing before weight is fully transferred.
#define WALK_SHIFT_STABLE_CYCLES  20   // ~200 ms dwell at 100 Hz

// Swing sequencing: fraction of swing elapsed before the stance knee starts bending.
// Ensures the float leg's hip pitch (step forward) completes before propulsion begins.
#define WALK_SWING_KNEE_DELAY     0.55f  // stance knee starts at 55% of swing duration

// Post-swing stability gate: after the swing timer ends, hold the landed pose and
// wait for the body to settle before advancing to the next SHIFT phase.
#define WALK_POST_SWING_STABLE_DEG    3.0f  // pitch and roll-error threshold [°]
#define WALK_POST_SWING_STABLE_CYCLES 8     // ~80 ms dwell at 100 Hz

// Walk-mode real-time IMU balance — corrections added on top of WalkController output
// every cycle. Applied on top of the walk pose; these trim each step to land level.
#define WALK_BAL_PITCH_GAIN    1.0f   // hip pitch trim  [rad / rad pitch error]
#define WALK_BAL_ROLL_GAIN     0.15f  // hip roll trim   [rad / rad roll error] — additive on top of walk lean, keep small
#define WALK_BAL_ANKLE_GAIN    0.7f   // ankle roll trim [rad / rad roll error]
#define WALK_BAL_ARM_DEG      25.0f   // shoulder FB correction [deg / rad pitch error] — visible relative to reduced ±10° gait swing
// EMA smoothing on corrections — removes digital stepping; 0.25 ≈ ~40 ms lag at 100 Hz
#define WALK_BAL_SMOOTH_ALPHA  0.25f

// ─── Balance enable ───────────────────────────────────────────────────────────
// Active body-plane maintenance using IMU. Requires USE_IMU=1 and ENABLE_GAIT=0.
// 0 = passive hold (T-pose / halt pose, no corrections)
// 1 = proportional corrections on hip pitch/roll (legs) + hip yaw damping (ch 10)
#define ENABLE_BALANCE  0

// ─── IK sagittal sway ────────────────────────────────────────────────────────
// Both feet fixed on ground. Hip oscillates forward/backward in the sagittal
// plane at IK_SWAY_PERIOD_S. IK recomputes hip_pitch + knee each cycle.
// Requires USE_IMU=0 or USE_IMU=1 (open-loop, no IMU feedback used).
// Mutually exclusive with ENABLE_GAIT, ENABLE_WALK, ENABLE_BALANCE, ENABLE_SINGLE_LEG_BALANCE.
#define ENABLE_IK_SWAY  1

#define IK_SWAY_PERIOD_S      4.0f   // autonomous oscillation period [s] (unused in joystick mode)
#define IK_SWAY_AMP_MM       20.0f   // hip forward/backward amplitude [mm]
#define IK_SWAY_KNEE_DEG     45.0f   // neutral knee bend [°]
#define IK_SWAY_SLEW_MM_PER_CYCLE  2.0f  // max hip travel per 100 Hz cycle [mm] — 200 mm/s max slew

// ─── Single-leg balance (right stance) ───────────────────────────────────────
// Balance on the right foot only while holding the left leg raised.
// Requires USE_IMU=1. Mutually exclusive with ENABLE_GAIT, ENABLE_WALK, ENABLE_BALANCE.
// Corrections applied every cycle via EMA-smoothed IMU:
//   pitch → right hip pitch
//   roll  → right hip roll + right ankle roll
//   arms  → shoulder FB (pitch) + shoulder lateral (roll)
#define ENABLE_SINGLE_LEG_BALANCE  0

// Float (left) leg — fixed raised pose held throughout
#define SLB_FLOAT_HIP_PITCH_DEG   20.0f   // hip pitch forward [°] — clears foot from ground
#define SLB_FLOAT_KNEE_DEG        30.0f   // knee bend [°] — extra foot clearance

// DC lateral lean: shifts CoM over the right (stance) foot before active balance begins.
// Same sign convention as WALK_SHIFT_RIGHT: right.hip_roll = -lean (negative), right.ankle_roll = +lean.
// Geometry: ~7° shifts CoM ~30 mm right at 250 mm CoM height. Single-leg needs a bit more.
#define SLB_STANCE_LEAN_DEG      10.0f   // pelvis lean right [°]

// Stance (right) leg gains
#define SLB_PITCH_GAIN       1.5f   // right hip pitch  [rad / rad pitch error]
#define SLB_ROLL_GAIN        1.5f   // right hip roll   [rad / rad roll error]
#define SLB_ANKLE_GAIN       1.0f   // right ankle roll [rad / rad roll error]

// Arm gains — larger than walk mode since arms are the primary upper-body corrector
#define SLB_ARM_PITCH_DEG   40.0f   // shoulder FB  [deg / rad pitch error]
#define SLB_ARM_ROLL_DEG    30.0f   // shoulder lat [deg / rad roll error]

// EMA smoothing on IMU — same lag as walk balance (~40 ms at 100 Hz)
#define SLB_SMOOTH_ALPHA     0.25f

// ─── IMU physical orientation ─────────────────────────────────────────────────
// X axis: right shoulder → left shoulder  (lateral)
// Y axis: feet → head                     (vertical / up when standing)
// Z axis: rear → front  (right-hand: X×Y) (sagittal / forward)
//
// Derived tilt axes:
//   body pitch (forward lean) = rotation around X → driven by gyro_x
//   body roll  (lateral lean) = rotation around Z → driven by gyro_z
//   body yaw   (twist)        = rotation around Y → driven by gyro_y
//
// Sign convention (+1 = "feels right on real hardware"):
#define IMU_PITCH_SIGN  (-1.0f)   // flipped: correction was inverted on real hardware
#define IMU_ROLL_SIGN   (+1.0f)   // +1 = positive when leaning right

// ─── Balance controller gains ─────────────────────────────────────────────────
#define BALANCE_PITCH_GAIN      1.2f   // [rad hip pitch / rad forward lean]
#define BALANCE_ROLL_GAIN       1.2f   // [rad hip roll  / rad lateral lean]
#define BALANCE_ANKLE_ROLL_GAIN 1.0f   // [rad ankle roll / rad lateral lean]
#define BALANCE_ARM_PITCH_GAIN  1.0f   // [deg shoulder FB / rad forward lean]
#define BALANCE_ARM_ROLL_GAIN   1.0f   // [deg shoulder lat / rad lateral lean]
#define BALANCE_YAW_RATE_GAIN   0.5f   // [rad servo / (rad/s yaw rate)]
#define BALANCE_YAW_MAX_DEG    45.0f   // clamp on hip yaw correction

// ─── IMU enable flag ─────────────────────────────────────────────────────────
// Set to 1 when MPU6050 is physically connected.
// Set to 0 to run CPG gait without accelerometer (IMU task and stabilizer
// are fully skipped; safety tilt cutoff is also disabled).
#define USE_IMU  1

// ─── MPU6050 ──────────────────────────────────────────────────────────────────
#define MPU6050_ADDR    0x68
#define IMU_SAMPLE_HZ   200     // Task frequency

// ─── PCA9685 ──────────────────────────────────────────────────────────────────
#define PCA9685_ADDR    0x40
#define PCA9685_FREQ_HZ 50      // Servo PWM frequency

// Servo channel mapping — wired to match legacy biped robot (robot_config.py)
//
//  PCA9685 ch │ Old joint name       │ CPG DOF       │ set_inverse │ halt°
//  ───────────┼──────────────────────┼───────────────┼─────────────┼──────
//   0         │ RightLeg.Ankle       │ R ankle_roll  │ False       │  90
//   1         │ RightLeg.Knee_Bottom │ R knee        │ False       │  80
//   2         │ RightLeg.Knee_Top    │ R hip_pitch   │ True        │ 100
//   3         │ RightLeg.Hip         │ R hip_roll    │ True        │ 100
//  12         │ LeftLeg.Hip          │ L hip_roll    │ False       │  80
//  13         │ LeftLeg.Knee_Top     │ L hip_pitch   │ False       │  90
//  14         │ LeftLeg.Knee_Bottom  │ L knee        │ True        │  90
//  15         │ LeftLeg.Ankle        │ L ankle_roll  │ True        │  70
#define SERVO_L_HIP_ROLL    12
#define SERVO_L_HIP_PITCH   13
#define SERVO_L_KNEE        14
#define SERVO_L_ANKLE_ROLL  15
#define SERVO_R_HIP_ROLL     3
#define SERVO_R_HIP_PITCH    2
#define SERVO_R_KNEE         1
#define SERVO_R_ANKLE_ROLL   0

// Upper-body servo channel mapping — Futaba S3003, channels 4–10
//
//  PCA9685 ch │ Joint                        │ T-pose° │ dir │ offset
//  ───────────┼──────────────────────────────┼─────────┼─────┼───────
//   4         │ R shoulder forward/backward  │   90    │ +1  │   0
//   5         │ R shoulder lateral elevation │   90    │ +1  │   0
//   6         │ R forearm lateral elevation  │   90    │ +1  │   0
//   7         │ L shoulder forward/backward  │   90    │ +1  │   0
//   8         │ L shoulder lateral elevation │   90    │ +1  │   0
//   9         │ L forearm lateral elevation  │   90    │ +1  │   0
//  10         │ Hip yaw (waist rotation)     │   90    │ +1  │   0
#define SERVO_R_SHOULDER_FB     4
#define SERVO_R_SHOULDER_LAT    5
#define SERVO_R_FOREARM_LAT     6
#define SERVO_L_SHOULDER_FB     7
#define SERVO_L_SHOULDER_LAT    8
#define SERVO_L_FOREARM_LAT     9
#define SERVO_HIP_YAW          10

// Upper-body servo directions (+1 = natural) and offsets (0 = centered/T-pose at 90°)
#define ARM_SERVO_DIR_R_SHOULDER_FB    (+1.0f)
#define ARM_SERVO_DIR_R_SHOULDER_LAT   (+1.0f)
#define ARM_SERVO_DIR_R_FOREARM_LAT    (+1.0f)
#define ARM_SERVO_DIR_L_SHOULDER_FB    (+1.0f)
#define ARM_SERVO_DIR_L_SHOULDER_LAT   (+1.0f)
#define ARM_SERVO_DIR_L_FOREARM_LAT    (+1.0f)
#define ARM_SERVO_DIR_HIP_YAW          (+1.0f)

#define ARM_SERVO_OFFSET_DEG_R_SHOULDER_FB    0.0f
#define ARM_SERVO_OFFSET_DEG_R_SHOULDER_LAT   0.0f
#define ARM_SERVO_OFFSET_DEG_R_FOREARM_LAT    0.0f
#define ARM_SERVO_OFFSET_DEG_L_SHOULDER_FB    0.0f
#define ARM_SERVO_OFFSET_DEG_L_SHOULDER_LAT   0.0f
#define ARM_SERVO_OFFSET_DEG_L_FOREARM_LAT    0.0f
#define ARM_SERVO_OFFSET_DEG_HIP_YAW          0.0f

// ─── CPG parameters ───────────────────────────────────────────────────────────
#define CPG_PERIOD_S              1.2f   // gait cycle period [s]
#define CPG_HIP_PITCH_AMP_DEG    31.0f  // hip pitch oscillation amplitude [°]
#define CPG_KNEE_AMP_DEG         31.0f  // knee oscillation amplitude [°]
#define CPG_HIP_ROLL_AMP_DEG     11.0f  // hip roll oscillation amplitude [°]
#define CPG_ANKLE_ROLL_AMP_DEG   11.0f  // ankle roll amplitude [°] — matched to hip roll

// Oscillator phase offsets [°]
// Base phase on BOTH legs' hip pitch: 0° = backward, 180° = forward.
#define CPG_HIP_PITCH_BASE_PHASE_DEG    0.0f   // 0° = forward
#define CPG_PHASE_HIP_PITCH_R_DEG     180.0f   // anti-phase L vs R
// Knee leads hip pitch by 30° for ground clearance before weight transfers.
#define CPG_PHASE_KNEE_OFFSET_DEG      30.0f   // +30° (π/6)
// Hip roll leads hip pitch by 120° so CoM shifts before swing leg lifts.
#define CPG_PHASE_HIP_ROLL_DEG        120.0f   // 2π/3 = 120°

// ─── Per-servo calibration ────────────────────────────────────────────────────
// Indexed by PCA9685 channel (0–15).  Unused channels are placeholders.
//
// Direction: +1 if set_inverse=False in robot_config.py, -1 if set_inverse=True.
// Offset:    (halt_deg - 90)  — shifts signed-center output to the physical halt pose.
//            Formula: physical_angle_0_180 = 90 + cpg_angle_signed * dir + offset
//
//  ch │ joint              │ dir  │ offset (halt-90)
//  ───┼────────────────────┼──────┼─────────────────
//   0 │ R ankle_roll       │ +1   │  0  (halt 90)
//   1 │ R knee             │ +1   │ -10 (halt 80)
//   2 │ R hip_pitch        │ -1   │ +10 (halt 100)
//   3 │ R hip_roll         │ -1   │ +10 (halt 100)
//  12 │ L hip_roll         │ +1   │ -10 (halt 80)
//  13 │ L hip_pitch        │ +1   │  0  (halt 90)
//  14 │ L knee             │ -1   │  0  (halt 90)
//  15 │ L ankle_roll       │ -1   │ -20 (halt 70)

#define SERVO_DIR_R_ANKLE_ROLL     (+1.0f)   // ch 0
#define SERVO_DIR_R_KNEE           (+1.0f)   // ch 1  set_inverse=False — increase = bending for right leg
#define SERVO_DIR_R_HIP_PITCH      (-1.0f)   // ch 2  set_inverse=True
#define SERVO_DIR_R_HIP_ROLL       (-1.0f)   // ch 3  set_inverse=True
#define SERVO_DIR_L_HIP_ROLL       (+1.0f)   // ch 12
#define SERVO_DIR_L_HIP_PITCH      (+1.0f)   // ch 13
#define SERVO_DIR_L_KNEE           (-1.0f)   // ch 14 set_inverse=True
#define SERVO_DIR_L_ANKLE_ROLL     (-1.0f)   // ch 15 set_inverse=True

#define SERVO_OFFSET_DEG_R_ANKLE_ROLL   (+8.0f)   // ch 0  matches R_HIP_ROLL offset → flat foot at halt
#define SERVO_OFFSET_DEG_R_KNEE        (-10.0f) // ch 1  halt 80
#define SERVO_OFFSET_DEG_R_HIP_PITCH   (+10.0f) // ch 2  halt 100
#define SERVO_OFFSET_DEG_R_HIP_ROLL     (+8.0f) // ch 3  wider stance — prevent foot collision
#define SERVO_OFFSET_DEG_L_HIP_ROLL     (-8.0f) // ch 12 wider stance — prevent foot collision
#define SERVO_OFFSET_DEG_L_HIP_PITCH     0.0f   // ch 13 halt 90
#define SERVO_OFFSET_DEG_L_KNEE          0.0f   // ch 14 halt 90
#define SERVO_OFFSET_DEG_L_ANKLE_ROLL   (-8.0f) // ch 15 matches L_HIP_ROLL offset → flat foot at halt

// ─── Per-leg hip roll DC trim ─────────────────────────────────────────────────
// 0 = oscillation centered at the halt pose (halt offsets = correct leg spread).
// The ±10° servo offsets on hip roll ARE the standing spread — do not cancel them.
#define CPG_HIP_ROLL_TRIM_L    0.0f
#define CPG_HIP_ROLL_TRIM_R    0.0f

// ─── Per-leg hip pitch stride trim ───────────────────────────────────────────
// DC offset added to hip pitch in the CPG (radians), independent of the servo
// physical halt offset. Use this to equalize stride length between legs without
// disturbing the standby pose.
// Positive = more forward lean on that leg, negative = less.
// Right leg has SERVO_OFFSET_DEG_R_HIP_PITCH=+10° which biases it forward vs
// left (0°). Counter it here with -10° = -0.175 rad on the right.
#define CPG_HIP_PITCH_TRIM_L_DEG    0.0f   // [°]
#define CPG_HIP_PITCH_TRIM_R_DEG  -10.0f  // [°]  cancels the +10° servo offset bias on right

// ─── Standing knee offset ─────────────────────────────────────────────────────
// DC knee bend applied as a CPG offset so legs don't lock straight under load.
// 0.30 rad ≈ 17°.  Old robot used ~45° (0.79 rad) knee bend from halt;
// increase this if the robot collapses at stance.
#define CPG_KNEE_STANDING_OFFSET_DEG   23.0f   // standing knee bend [°] — crouch keeps legs from locking straight

// ─── Safety cutoff ────────────────────────────────────────────────────────────
// If the IMU detects tilt beyond this limit (fall / tip-over), the CPG task
// halts all servo outputs immediately and suspends itself.
#define SAFETY_TILT_LIMIT_DEG   50.0f   // fall/tip-over cutoff [°] — halts all servos and suspends task

// ─── IMU stabilizer thresholds ────────────────────────────────────────────────
#define STAB_PITCH_THRESHOLD_DEG  0.6f    // pitch dead-band [°] — react from the first sign of tilt
#define STAB_ROLL_THRESHOLD_DEG   0.6f    // roll  dead-band [°]
#define STAB_PITCH_GAIN           0.4f
#define STAB_ROLL_GAIN            0.4f
#define COMPLEMENTARY_ALPHA       0.98f

// ─── FreeRTOS task config ─────────────────────────────────────────────────────
#define TASK_IMU_HZ        200
#define TASK_CPG_HZ        100
#define TASK_TELEMETRY_HZ   50

#define TASK_IMU_STACK      4096
#define TASK_CPG_STACK      8192
#define TASK_TELEMETRY_STACK 8192

#define TASK_IMU_PRIORITY      3
#define TASK_CPG_PRIORITY      2
#define TASK_TELEMETRY_PRIORITY 1

// ─── WiFi / WebSocket ─────────────────────────────────────────────────────────
// Credentials live in secrets.h (git-ignored). Copy secrets.h.example → secrets.h and fill in.
#include "secrets.h"
#define WS_PORT         81
#define MDNS_HOSTNAME   "optimus"   // reachable as optimus.local on the LAN
