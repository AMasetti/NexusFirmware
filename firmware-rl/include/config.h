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
#define SERVO_PWM_MIN    600   // µs
#define SERVO_PWM_MAX   2650   // µs

// Servo PWM range (Futaba S3003 arms)
#define ARM_SERVO_PWM_MIN    900   // µs
#define ARM_SERVO_PWM_MAX   2100   // µs

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

#define SERVO_OFFSET_DEG_R_ANKLE_ROLL   (+8.0f)
#define SERVO_OFFSET_DEG_R_KNEE        (-10.0f)
#define SERVO_OFFSET_DEG_R_HIP_PITCH   (+10.0f)
#define SERVO_OFFSET_DEG_R_HIP_ROLL     (+8.0f)
#define SERVO_OFFSET_DEG_L_HIP_ROLL     (-8.0f)
#define SERVO_OFFSET_DEG_L_HIP_PITCH     0.0f
#define SERVO_OFFSET_DEG_L_KNEE          0.0f
#define SERVO_OFFSET_DEG_L_ANKLE_ROLL   (-8.0f)

// Arm servos: centered at 90°, no direction flip needed for RL mode
// (PC sends absolute angles; arms receive them as-is)
#define ARM_SERVO_DIR       (+1.0f)
#define ARM_SERVO_OFFSET     0.0f

// ─── Per-joint soft limits [degrees] ─────────────────────────────────────────
// ESP32 clamps incoming angles before writing — last line of defence.
#define JOINT_HIP_ROLL_MIN_DEG      -45.0f
#define JOINT_HIP_ROLL_MAX_DEG       45.0f
#define JOINT_HIP_PITCH_MIN_DEG     -45.0f
#define JOINT_HIP_PITCH_MAX_DEG      45.0f
#define JOINT_KNEE_MIN_DEG          -70.0f
#define JOINT_KNEE_MAX_DEG           70.0f
#define JOINT_ANKLE_ROLL_MIN_DEG    -90.0f
#define JOINT_ANKLE_ROLL_MAX_DEG     90.0f
#define JOINT_SHOULDER_FB_MIN_DEG  -120.0f
#define JOINT_SHOULDER_FB_MAX_DEG   120.0f
#define JOINT_SHOULDER_LAT_MIN_DEG -120.0f
#define JOINT_SHOULDER_LAT_MAX_DEG  120.0f
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
