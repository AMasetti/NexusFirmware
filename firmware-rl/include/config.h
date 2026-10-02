#pragma once

// ─── Robot geometry ───────────────────────────────────────────────────────────
// Same physical robot as firmware/. These values are reference-only here
// (no IK runs on the ESP32 in RL mode).
#define L1_MM     90.0f
#define L2_MM     90.0f
#define L3_MM     30.0f

// ─── I2C ──────────────────────────────────────────────────────────────────────
#define I2C_SDA_PIN      8
#define I2C_SCL_PIN      9
#define I2C_FREQ_HZ     400000

// ─── IMU ──────────────────────────────────────────────────────────────────────
#define MPU6050_ADDR        0x68
#define IMU_SAMPLE_HZ       100     // complementary filter rate
#define COMPLEMENTARY_ALPHA 0.98f   // matches firmware/ and sim CPG

// IMU physical axis signs (matched to firmware/include/config.h)
#define IMU_PITCH_SIGN  (-1.0f)
#define IMU_ROLL_SIGN   (+1.0f)

// ─── PCA9685 ──────────────────────────────────────────────────────────────────
#define PCA9685_ADDR    0x40
#define PCA9685_FREQ_HZ 50

// Servo PWM range (MG995 legs)
#define SERVO_MIN_DEG   -90.0f
#define SERVO_MAX_DEG    90.0f
#define SERVO_PWM_MIN    600   // µs
#define SERVO_PWM_MAX   2650   // µs

// Servo PWM range (Futaba S3003 arms)
// Widened 2026-08-11: 900–2100 only reached ~120 deg of the S3003's ~180 deg
// travel, so shoulder_lat stalled ~45 deg short of hanging fully down on both
// arms. Extended 150 us per side (= 22.5 deg each at 6.667 us/deg) to recover
// 45 deg of total range. Affects ALL arm channels, not just shoulder_lat.
#define ARM_SERVO_PWM_MIN    750   // µs
#define ARM_SERVO_PWM_MAX   2250   // µs

// ─── Servo channel mapping ────────────────────────────────────────────────────
// Must match firmware/include/config.h exactly.
#define SERVO_R_ANKLE_ROLL   0
#define SERVO_R_KNEE         1
#define SERVO_R_HIP_PITCH    2
#define SERVO_R_HIP_ROLL     3
#define SERVO_R_SHOULDER_FB  4
#define SERVO_R_SHOULDER_LAT 5
#define SERVO_R_FOREARM_LAT  6
#define SERVO_L_SHOULDER_FB  7
#define SERVO_L_SHOULDER_LAT 8
#define SERVO_L_FOREARM_LAT  9
#define SERVO_HIP_YAW       10
#define SERVO_L_HIP_ROLL    12
#define SERVO_L_HIP_PITCH   13
#define SERVO_L_KNEE        14
#define SERVO_L_ANKLE_ROLL  15

// Per-servo direction and halt offset — must match firmware/include/config.h.
// physical_deg = 90 + angle_rad * RAD2DEG * dir + offset_deg
#define SERVO_DIR_R_ANKLE_ROLL    (+1.0f)
#define SERVO_DIR_R_KNEE          (+1.0f)
#define SERVO_DIR_R_HIP_PITCH     (-1.0f)
#define SERVO_DIR_R_HIP_ROLL      (-1.0f)
#define SERVO_DIR_L_HIP_ROLL      (+1.0f)
#define SERVO_DIR_L_HIP_PITCH     (+1.0f)
#define SERVO_DIR_L_KNEE          (-1.0f)
#define SERVO_DIR_L_ANKLE_ROLL    (-1.0f)

// Balanced with L (-5) — R was +15, and unequal magnitudes on a mirrored pair
// (SERVO_DIR_R = +1, L = -1) leave the feet tilted relative to each other.
#define SERVO_OFFSET_DEG_R_ANKLE_ROLL    (+8.0f)   // JOINTS.md ch0: flat foot
// Restored from docs/JOINTS.md "Servo direction and offset — leg channels",
// which is the bench-verified T-pose centring table. firmware-rl had drifted:
// R knee was +10 (table says -10) and L knee +10 (table says 0). Because
// SERVO_DIR_R_KNEE = +1 and SERVO_DIR_L_KNEE = -1, an identical offset on the
// pair pushes the two knees in OPPOSITE physical directions — which is why the
// right leg flexed visibly more than the left for the same command.
#define SERVO_OFFSET_DEG_R_KNEE         (-10.0f)  // JOINTS.md ch1: halt 80 deg
#define SERVO_OFFSET_DEG_R_HIP_PITCH    (+10.0f)  // JOINTS.md ch2: halt 100 deg
// Legs sat too wide at +20/-10. Halved both magnitudes to close the stance;
// these are the axis that splays the legs (DIR is -1 on R, +1 on L, so the
// opposite signs push both outward). Tune on hardware.
// Balanced 2026-08-15: R was +10 and L was -5. With SERVO_DIR_R = -1 and
// SERVO_DIR_L = +1 those unequal magnitudes leave a constant sideways bias,
// and on hardware the robot only ever leaned onto the right foot — the left
// leg never received the body weight. Equal magnitudes remove the bias.
#define SERVO_OFFSET_DEG_R_HIP_ROLL      (+8.0f)  // was +10, before that +20 / -20
#define SERVO_OFFSET_DEG_L_HIP_ROLL      (-8.0f)  // was -5
#define SERVO_OFFSET_DEG_L_HIP_PITCH     (0.0f)   // JOINTS.md ch13: halt 90 deg
#define SERVO_OFFSET_DEG_L_KNEE          (0.0f)   // JOINTS.md ch14: halt 90 deg
#define SERVO_OFFSET_DEG_L_ANKLE_ROLL   (-8.0f)   // JOINTS.md ch15: flat foot

// Arm servo dirs — +1 normal, -1 inverted (servo mounted mirrored).
// R and L shoulder_lat are mirrored: same signal must move both arms down.
// Verified 2026-08-11: R needs -90 deg to hang down, L needs +90 deg to hang down
// → they are physically inverted relative to each other.
#define ARM_SERVO_DIR_L_SHOULDER_FB   (+1.0f)
#define ARM_SERVO_DIR_R_SHOULDER_FB   (+1.0f)
#define ARM_SERVO_DIR_L_SHOULDER_LAT  (+1.0f)
#define ARM_SERVO_DIR_R_SHOULDER_LAT  (+1.0f)
#define ARM_SERVO_DIR_L_FOREARM_LAT   (+1.0f)
#define ARM_SERVO_DIR_R_FOREARM_LAT   (+1.0f)
#define ARM_SERVO_DIR_HIP_YAW         (+1.0f)

// Arm servo offsets [degrees] — applied after dir: physical = angle_rad*RAD2DEG*dir + offset.
// write_arm_servo maps [-90,+90] → [PWM_MIN, PWM_MAX], center (0 deg) = T-pose.
// offset = -90 → 0 rad input = arms hanging at sides.
#define ARM_SERVO_OFFSET_DEG_L_SHOULDER_FB   (+15.0f)
#define ARM_SERVO_OFFSET_DEG_R_SHOULDER_FB   (+5.0f)
#define ARM_SERVO_OFFSET_DEG_L_SHOULDER_LAT   (0.0f)
#define ARM_SERVO_OFFSET_DEG_R_SHOULDER_LAT   (0.0f)
#define ARM_SERVO_OFFSET_DEG_L_FOREARM_LAT   (-20.0f)
#define ARM_SERVO_OFFSET_DEG_R_FOREARM_LAT   (+5.0f)
#define ARM_SERVO_OFFSET_DEG_HIP_YAW          0.0f

// ─── Per-joint soft limits [degrees] ─────────────────────────────────────────
// ESP32 clamps incoming angles before writing — last line of defence.
#define JOINT_HIP_ROLL_MIN_DEG      -45.0f
#define JOINT_HIP_ROLL_MAX_DEG       45.0f
// RL policy (run_27) commands up to 55.5 deg here — Servo-Knee-*-Top maps to
// hip_pitch, and the old +/-45 clamp truncated the peak of every stride.
#define JOINT_HIP_PITCH_MIN_DEG    -105.0f
#define JOINT_HIP_PITCH_MAX_DEG     105.0f
#define JOINT_KNEE_MIN_DEG          -70.0f
#define JOINT_KNEE_MAX_DEG           70.0f
#define JOINT_ANKLE_ROLL_MIN_DEG    -90.0f
#define JOINT_ANKLE_ROLL_MAX_DEG     90.0f
#define JOINT_SHOULDER_FB_MIN_DEG  -120.0f
#define JOINT_SHOULDER_FB_MAX_DEG   120.0f
#define JOINT_SHOULDER_LAT_MIN_DEG -180.0f
#define JOINT_SHOULDER_LAT_MAX_DEG  180.0f
#define JOINT_FOREARM_LAT_MIN_DEG   -90.0f
#define JOINT_FOREARM_LAT_MAX_DEG    90.0f
#define JOINT_HIP_YAW_MIN_DEG       -45.0f
#define JOINT_HIP_YAW_MAX_DEG        45.0f

// ─── Safety cutoff ────────────────────────────────────────────────────────────
// If IMU detects tilt beyond this, servo output freezes and ESP32 halts.
// The PC runner also has its own safety, but this is the hardware last resort.
#define SAFETY_TILT_LIMIT_DEG   50.0f

// ─── RL servo output timeout ──────────────────────────────────────────────────
// If the PC stops sending commands (cable pull, crash), hold last pose for
// SERVO_TIMEOUT_MS then freeze servos to prevent runaway.
#define SERVO_TIMEOUT_MS   200

// ─── WiFi / WebSocket ─────────────────────────────────────────────────────────
#include "secrets.h"
#define WS_PORT         81
#define MDNS_HOSTNAME   "optimus-rl"   // optimus-rl.local on LAN

// ─── FreeRTOS task config ─────────────────────────────────────────────────────
#define TASK_IMU_HZ          100
#define TASK_SERVO_HZ         50   // matches PCA9685_FREQ_HZ and sim ctrl_dt
#define TASK_TELEMETRY_HZ     50

#define TASK_IMU_STACK        4096
#define TASK_SERVO_STACK      4096
#define TASK_TELEMETRY_STACK  8192

#define TASK_IMU_PRIORITY       3
#define TASK_SERVO_PRIORITY     2
#define TASK_TELEMETRY_PRIORITY 1
