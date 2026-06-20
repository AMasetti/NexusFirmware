#pragma once
#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>
#include "../../include/config.h"
#include "../gait/stabilizer.h"   // IMUEstimate; also pulls cpg.h → JointAngles / LegAngles
#include "../gait/cpg.h"
#include "../gait/walk.h"
#include "../imu/mpu6050.h"
#include "../servo/pca9685.h"

static constexpr float RAD2DEG = 57.29577951f;
static constexpr float DEG2RAD =  0.017453293f;

// ─── Calibration tables ───────────────────────────────────────────────────────
// Indexed by PCA9685 channel (0–15).  ch 10 (hip_yaw) is driven via set_pulse_us.
static constexpr float SERVO_DIR[16] = {
    /* ch  0 R_ankle_roll */ SERVO_DIR_R_ANKLE_ROLL,
    /* ch  1 R_knee       */ SERVO_DIR_R_KNEE,
    /* ch  2 R_hip_pitch  */ SERVO_DIR_R_HIP_PITCH,
    /* ch  3 R_hip_roll   */ SERVO_DIR_R_HIP_ROLL,
    /* ch  4 */  0.0f,
    /* ch  5 */  0.0f,
    /* ch  6 */  0.0f,
    /* ch  7 */  0.0f,
    /* ch  8 */  0.0f,
    /* ch  9 */  0.0f,
    /* ch 10 */  0.0f,
    /* ch 11 */  0.0f,
    /* ch 12 L_hip_roll   */ SERVO_DIR_L_HIP_ROLL,
    /* ch 13 L_hip_pitch  */ SERVO_DIR_L_HIP_PITCH,
    /* ch 14 L_knee       */ SERVO_DIR_L_KNEE,
    /* ch 15 L_ankle_roll */ SERVO_DIR_L_ANKLE_ROLL,
};
static constexpr float SERVO_OFFSET_DEG[16] = {
    /* ch  0 */ SERVO_OFFSET_DEG_R_ANKLE_ROLL,
    /* ch  1 */ SERVO_OFFSET_DEG_R_KNEE,
    /* ch  2 */ SERVO_OFFSET_DEG_R_HIP_PITCH,
    /* ch  3 */ SERVO_OFFSET_DEG_R_HIP_ROLL,
    /* ch  4 */  0.0f,
    /* ch  5 */  0.0f,
    /* ch  6 */  0.0f,
    /* ch  7 */  0.0f,
    /* ch  8 */  0.0f,
    /* ch  9 */  0.0f,
    /* ch 10 */  0.0f,
    /* ch 11 */  0.0f,
    /* ch 12 */ SERVO_OFFSET_DEG_L_HIP_ROLL,
    /* ch 13 */ SERVO_OFFSET_DEG_L_HIP_PITCH,
    /* ch 14 */ SERVO_OFFSET_DEG_L_KNEE,
    /* ch 15 */ SERVO_OFFSET_DEG_L_ANKLE_ROLL,
};
static constexpr uint8_t LEG_CH[2][4] = {
    { SERVO_L_HIP_ROLL, SERVO_L_HIP_PITCH, SERVO_L_KNEE, SERVO_L_ANKLE_ROLL },
    { SERVO_R_HIP_ROLL, SERVO_R_HIP_PITCH, SERVO_R_KNEE, SERVO_R_ANKLE_ROLL },
};

// ─── Shared globals (defined in main.cpp) ────────────────────────────────────
extern SemaphoreHandle_t    state_mutex;
extern volatile IMUEstimate g_imu_estimate;
extern volatile LegAngles   g_leg_angles;
extern volatile float       g_time_s;
extern volatile float       g_sway_target_mm;

extern MPU6050        imu;
extern PCA9685        servos;
extern CPGOscillator  cpg;
extern Stabilizer     stabilizer;
extern WalkController walk_ctrl;

// ─── Helpers (defined in main.cpp) ───────────────────────────────────────────
void write_leg(const JointAngles& q, uint8_t leg_idx);
void write_arms_tpose();
