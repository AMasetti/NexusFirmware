#include <Arduino.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#include "../include/config.h"
#include "gait/cpg.h"
#include "gait/stabilizer.h"
#include "gait/walk.h"
#include "imu/mpu6050.h"
#include "servo/pca9685.h"
#include "comms/telemetry.h"

// ─── Experiment tasks ─────────────────────────────────────────────────────────
// Each experiment lives in src/experiments/exp_*.cpp.
// Only the task function(s) matching the active flag are linked.
#include "experiments/exp_gait.h"
#include "experiments/exp_walk.h"
#include "experiments/exp_balance.h"
#include "experiments/exp_slb.h"
#include "experiments/exp_ik_sway.h"
#include "experiments/shared.h"    // also supplies RAD2DEG / DEG2RAD

// ─── Compile-time flag validation ────────────────────────────────────────────
static_assert(!(ENABLE_BALANCE && !USE_IMU),   "ENABLE_BALANCE requires USE_IMU=1");
static_assert(!(ENABLE_BALANCE && ENABLE_GAIT), "ENABLE_BALANCE requires ENABLE_GAIT=0");
static_assert(!(ENABLE_WALK    && !USE_IMU),    "ENABLE_WALK requires USE_IMU=1");
static_assert(!(ENABLE_WALK && ENABLE_GAIT),    "ENABLE_WALK requires ENABLE_GAIT=0");
static_assert(!(ENABLE_WALK && ENABLE_BALANCE), "ENABLE_WALK and ENABLE_BALANCE are mutually exclusive");
static_assert(!(ENABLE_SINGLE_LEG_BALANCE && !USE_IMU),       "ENABLE_SINGLE_LEG_BALANCE requires USE_IMU=1");
static_assert(!(ENABLE_SINGLE_LEG_BALANCE && ENABLE_GAIT),    "ENABLE_SINGLE_LEG_BALANCE requires ENABLE_GAIT=0");
static_assert(!(ENABLE_SINGLE_LEG_BALANCE && ENABLE_WALK),    "ENABLE_SINGLE_LEG_BALANCE requires ENABLE_WALK=0");
static_assert(!(ENABLE_SINGLE_LEG_BALANCE && ENABLE_BALANCE), "ENABLE_SINGLE_LEG_BALANCE requires ENABLE_BALANCE=0");
static_assert(!(ENABLE_IK_SWAY && ENABLE_GAIT),              "ENABLE_IK_SWAY requires ENABLE_GAIT=0");
static_assert(!(ENABLE_IK_SWAY && ENABLE_WALK),              "ENABLE_IK_SWAY requires ENABLE_WALK=0");
static_assert(!(ENABLE_IK_SWAY && ENABLE_BALANCE),           "ENABLE_IK_SWAY requires ENABLE_BALANCE=0");
static_assert(!(ENABLE_IK_SWAY && ENABLE_SINGLE_LEG_BALANCE),"ENABLE_IK_SWAY requires ENABLE_SINGLE_LEG_BALANCE=0");

// ─── Shared globals ───────────────────────────────────────────────────────────
// Declared extern in experiments/shared.h; defined here (single translation unit).
MPU6050        imu;
PCA9685        servos;
CPGOscillator  cpg;
Stabilizer     stabilizer;
WalkController walk_ctrl;
Telemetry      telemetry;

SemaphoreHandle_t   state_mutex;
volatile IMUEstimate g_imu_estimate = {0.0f, 0.0f, 0.0f};
volatile LegAngles   g_leg_angles;
volatile float       g_time_s = 0.0f;
volatile float       g_sway_target_mm = 0.0f;

// ─── Servo output helpers ─────────────────────────────────────────────────────
// write_leg and write_arms_tpose have external linkage so experiments can use them.
// Calibration tables are also in shared.h (static constexpr, each TU gets a copy).

void write_leg(const JointAngles& q, uint8_t leg_idx) {
    const float raw[4] = { q.hip_roll, q.hip_pitch, q.knee, q.ankle_roll };
    for (int j = 0; j < 4; ++j) {
        uint8_t ch  = LEG_CH[leg_idx][j];
        float   deg = raw[j] * RAD2DEG * SERVO_DIR[ch] + SERVO_OFFSET_DEG[ch];
        servos.set_angle(ch, deg);
    }
}

void write_arms_tpose() {
    const uint8_t arm_chs[7] = {
        SERVO_R_SHOULDER_FB,  SERVO_R_SHOULDER_LAT,  SERVO_R_FOREARM_LAT,
        SERVO_L_SHOULDER_FB,  SERVO_L_SHOULDER_LAT,  SERVO_L_FOREARM_LAT,
        SERVO_HIP_YAW,
    };
    const uint16_t center_us = (ARM_SERVO_PWM_MIN + ARM_SERVO_PWM_MAX) / 2;
    for (uint8_t i = 0; i < 7; ++i) {
        servos.set_pulse_us(arm_chs[i], center_us);
    }
}

// ─── Per-joint soft limits [radians] ─────────────────────────────────────────
struct JointLimits { float min_rad; float max_rad; };

// Leg joints [0–3]: hip_roll, hip_pitch, knee, ankle_roll
static constexpr JointLimits LEG_LIMITS[4] = {
    { JOINT_HIP_ROLL_MIN_DEG    * DEG2RAD, JOINT_HIP_ROLL_MAX_DEG    * DEG2RAD },
    { JOINT_HIP_PITCH_MIN_DEG   * DEG2RAD, JOINT_HIP_PITCH_MAX_DEG   * DEG2RAD },
    { JOINT_KNEE_MIN_DEG        * DEG2RAD, JOINT_KNEE_MAX_DEG         * DEG2RAD },
    { JOINT_ANKLE_ROLL_MIN_DEG  * DEG2RAD, JOINT_ANKLE_ROLL_MAX_DEG   * DEG2RAD },
};

// Arm channels with their limits and PCA9685 channel number
struct ArmJoint { uint8_t ch; float dir; float offset_deg; float min_rad; float max_rad; };
static constexpr ArmJoint ARM_JOINTS[] = {
    { SERVO_R_SHOULDER_FB,  ARM_SERVO_DIR_R_SHOULDER_FB,  ARM_SERVO_OFFSET_DEG_R_SHOULDER_FB,  JOINT_SHOULDER_FB_MIN_DEG  * DEG2RAD, JOINT_SHOULDER_FB_MAX_DEG  * DEG2RAD },
    { SERVO_R_SHOULDER_LAT, ARM_SERVO_DIR_R_SHOULDER_LAT, ARM_SERVO_OFFSET_DEG_R_SHOULDER_LAT, JOINT_SHOULDER_LAT_MIN_DEG * DEG2RAD, JOINT_SHOULDER_LAT_MAX_DEG * DEG2RAD },
    { SERVO_R_FOREARM_LAT,  ARM_SERVO_DIR_R_FOREARM_LAT,  ARM_SERVO_OFFSET_DEG_R_FOREARM_LAT,  JOINT_FOREARM_LAT_MIN_DEG  * DEG2RAD, JOINT_FOREARM_LAT_MAX_DEG  * DEG2RAD },
    { SERVO_L_SHOULDER_FB,  ARM_SERVO_DIR_L_SHOULDER_FB,  ARM_SERVO_OFFSET_DEG_L_SHOULDER_FB,  JOINT_SHOULDER_FB_MIN_DEG  * DEG2RAD, JOINT_SHOULDER_FB_MAX_DEG  * DEG2RAD },
    { SERVO_L_SHOULDER_LAT, ARM_SERVO_DIR_L_SHOULDER_LAT, ARM_SERVO_OFFSET_DEG_L_SHOULDER_LAT, JOINT_SHOULDER_LAT_MIN_DEG * DEG2RAD, JOINT_SHOULDER_LAT_MAX_DEG * DEG2RAD },
    { SERVO_L_FOREARM_LAT,  ARM_SERVO_DIR_L_FOREARM_LAT,  ARM_SERVO_OFFSET_DEG_L_FOREARM_LAT,  JOINT_FOREARM_LAT_MIN_DEG  * DEG2RAD, JOINT_FOREARM_LAT_MAX_DEG  * DEG2RAD },
    { SERVO_HIP_YAW,        ARM_SERVO_DIR_HIP_YAW,        ARM_SERVO_OFFSET_DEG_HIP_YAW,        JOINT_HIP_YAW_MIN_DEG      * DEG2RAD, JOINT_HIP_YAW_MAX_DEG      * DEG2RAD },
};

// ─── Joint name → leg index + joint index ─────────────────────────────────────
// For leg joints: returns true, sets leg (0=L,1=R) and jnt (0–3).
// For arm joints: returns false with leg=0xFF as sentinel — caller checks arm map.
static bool resolve_joint(const char* name, uint8_t& leg, uint8_t& jnt) {
    struct { const char* n; uint8_t l; uint8_t j; } LEG_MAP[] = {
        { "l_hip_roll",   0, 0 }, { "l_hip_pitch",  0, 1 },
        { "l_knee",       0, 2 }, { "l_ankle_roll", 0, 3 },
        { "r_hip_roll",   1, 0 }, { "r_hip_pitch",  1, 1 },
        { "r_knee",       1, 2 }, { "r_ankle_roll", 1, 3 },
    };
    for (const auto& e : LEG_MAP) {
        if (strcmp(name, e.n) == 0) { leg = e.l; jnt = e.j; return true; }
    }
    return false;
}

// Returns index into ARM_JOINTS[], or -1 if not found.
static int resolve_arm_joint(const char* name) {
    static const char* ARM_NAMES[] = {
        "r_shoulder_fb", "r_shoulder_lat", "r_forearm_lat",
        "l_shoulder_fb", "l_shoulder_lat", "l_forearm_lat",
        "hip_yaw",
    };
    for (int i = 0; i < 7; ++i) {
        if (strcmp(name, ARM_NAMES[i]) == 0) return i;
    }
    return -1;
}

// ─── Telemetry parameter callback ─────────────────────────────────────────────
static void on_param(const char* cmd, const char* joint, float value) {
    if (strcmp(cmd, "set_joint") == 0 && joint[0] != '\0') {
        uint8_t leg = 0, jnt_idx = 0;
        if (resolve_joint(joint, leg, jnt_idx)) {
            // Leg joint — clamp, write servo, update shared state
            JointAngles q;
            xSemaphoreTake(state_mutex, portMAX_DELAY);
            if (leg == 0) {
                memcpy(&q, (const void*)&g_leg_angles.left,  sizeof(JointAngles));
            } else {
                memcpy(&q, (const void*)&g_leg_angles.right, sizeof(JointAngles));
            }
            xSemaphoreGive(state_mutex);

            const JointLimits& lim = LEG_LIMITS[jnt_idx];
            float clamped = fmaxf(lim.min_rad, fminf(lim.max_rad, value));
            float* fields[4] = { &q.hip_roll, &q.hip_pitch, &q.knee, &q.ankle_roll };
            *fields[jnt_idx] = clamped;
            write_leg(q, leg);

            xSemaphoreTake(state_mutex, portMAX_DELAY);
            if (leg == 0) {
                g_leg_angles.left.hip_roll   = q.hip_roll;
                g_leg_angles.left.hip_pitch  = q.hip_pitch;
                g_leg_angles.left.knee       = q.knee;
                g_leg_angles.left.ankle_roll = q.ankle_roll;
            } else {
                g_leg_angles.right.hip_roll   = q.hip_roll;
                g_leg_angles.right.hip_pitch  = q.hip_pitch;
                g_leg_angles.right.knee       = q.knee;
                g_leg_angles.right.ankle_roll = q.ankle_roll;
            }
            xSemaphoreGive(state_mutex);
        } else {
            // Arm joint — clamp and write directly to PCA9685
            int arm_idx = resolve_arm_joint(joint);
            if (arm_idx >= 0) {
                const ArmJoint& aj = ARM_JOINTS[arm_idx];
                float clamped = fmaxf(aj.min_rad, fminf(aj.max_rad, value));
                float deg = clamped * RAD2DEG * aj.dir + aj.offset_deg + 90.0f;
                servos.set_angle(aj.ch, deg);
            }
        }
    } else if (strcmp(cmd, "set_period") == 0) {
        cpg.set_period(value);
    } else if (strcmp(cmd, "set_amp_hp") == 0) {
        CPGParams p = cpg.params();
        cpg.set_amplitude(p.hip_roll_amp, value, p.knee_amp, p.ankle_roll_amp);
    } else if (strcmp(cmd, "set_amp_hr") == 0) {
        CPGParams p = cpg.params();
        cpg.set_amplitude(value, p.hip_pitch_amp, p.knee_amp, p.ankle_roll_amp);
    } else if (strcmp(cmd, "set_amp_k") == 0) {
        CPGParams p = cpg.params();
        cpg.set_amplitude(p.hip_roll_amp, p.hip_pitch_amp, value, p.ankle_roll_amp);
    } else if (strcmp(cmd, "set_amp_ar") == 0) {
        CPGParams p = cpg.params();
        cpg.set_amplitude(p.hip_roll_amp, p.hip_pitch_amp, p.knee_amp, value);
    } else if (strcmp(cmd, "set_sway_target") == 0) {
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        g_sway_target_mm = value;
        xSemaphoreGive(state_mutex);
    } else if (strcmp(cmd, "set_neutral") == 0) {
        servos.set_all_neutral();
    } else if (strcmp(cmd, "calibrate_imu") == 0) {
#if USE_IMU
        imu.calibrate(200);
#endif
    }
}

// ─── Task: IMU @ 200 Hz ───────────────────────────────────────────────────────
#if USE_IMU
static void task_imu(void* /*arg*/) {
    const TickType_t period = pdMS_TO_TICKS(1000 / TASK_IMU_HZ);
    const float dt = 1.0f / TASK_IMU_HZ;
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        if (imu.update(dt)) {
            IMUEstimate est = imu.estimate();
            xSemaphoreTake(state_mutex, portMAX_DELAY);
            memcpy((void*)&g_imu_estimate, &est, sizeof(IMUEstimate));
            xSemaphoreGive(state_mutex);
        }
        vTaskDelayUntil(&last_wake, period);
    }
}
#endif  // USE_IMU

// ─── Task: Telemetry ──────────────────────────────────────────────────────────
// ws->loop() (command receive) runs at TASK_TELEMETRY_HZ (50 Hz).
// State broadcast runs at 10 Hz to avoid flooding the WebSocket client.
#define TASK_TELEMETRY_BROADCAST_HZ  10
static void task_telemetry(void* /*arg*/) {
    const TickType_t period = pdMS_TO_TICKS(1000 / TASK_TELEMETRY_HZ);
    TickType_t last_wake = xTaskGetTickCount();
    uint32_t broadcast_divider = 0;

    for (;;) {
        telemetry.loop();   // process incoming commands at full rate

        // Broadcast state at reduced rate
        if (++broadcast_divider >= (TASK_TELEMETRY_HZ / TASK_TELEMETRY_BROADCAST_HZ)) {
            broadcast_divider = 0;

            RobotState state;
            xSemaphoreTake(state_mutex, portMAX_DELAY);
            memcpy(&state.imu, (const void*)&g_imu_estimate, sizeof(IMUEstimate));
            state.legs = {
                {g_leg_angles.left.hip_roll,  g_leg_angles.left.hip_pitch,
                 g_leg_angles.left.knee,      g_leg_angles.left.ankle_roll},
                {g_leg_angles.right.hip_roll, g_leg_angles.right.hip_pitch,
                 g_leg_angles.right.knee,     g_leg_angles.right.ankle_roll}
            };
            xSemaphoreGive(state_mutex);
            // raw_ is only written in task_imu; reading here without mutex is safe
            // (worst case we get a slightly stale sample, acceptable at 10 Hz)
            state.raw = imu.raw();

            state.cpg          = cpg.params();
            state.timestamp_ms = (uint32_t)millis();
            telemetry.send_state(state);
        }

        vTaskDelayUntil(&last_wake, period);
    }
}

// ─── I2C scanner ──────────────────────────────────────────────────────────────
static void scan_i2c() {
    Serial.println("[I2C] Scanning bus...");
    uint8_t found = 0;
    for (uint8_t addr = 1; addr < 127; ++addr) {
        Wire.beginTransmission(addr);
        uint8_t err = Wire.endTransmission();
        if (err == 0) {
            Serial.printf("[I2C]   device at 0x%02X", addr);
            if (addr == 0x40) Serial.print("  ← PCA9685 (servos)");
            if (addr == 0x68) Serial.print("  ← MPU6050 (IMU)");
            if (addr == 0x70) Serial.print("  ← PCA9685 all-call");
            Serial.println();
            ++found;
        }
    }
    if (found == 0)
        Serial.println("[I2C]   no devices found — check SDA/SCL wiring and 3.3 V pull-ups");
    else
        Serial.printf("[I2C]   %u device(s) found\n", found);
}

// ─── Direct servo sweep test ──────────────────────────────────────────────────
static void servo_sweep_test() {
    Serial.println("[TEST] Starting servo sweep (3 sweeps per channel)...");
    const uint8_t test_channels[8] = {
        SERVO_R_ANKLE_ROLL, SERVO_R_KNEE, SERVO_R_HIP_PITCH, SERVO_R_HIP_ROLL,
        SERVO_L_HIP_ROLL,   SERVO_L_HIP_PITCH, SERVO_L_KNEE, SERVO_L_ANKLE_ROLL
    };
    const char* names[8] = {
        "R_ankle_roll(0)", "R_knee(1)", "R_hip_pitch(2)", "R_hip_roll(3)",
        "L_hip_roll(12)",  "L_hip_pitch(13)", "L_knee(14)", "L_ankle_roll(15)"
    };
    for (int i = 0; i < 8; ++i) {
        Serial.printf("[TEST]   ch %2u  %s\n", test_channels[i], names[i]);
        for (int rep = 0; rep < 3; ++rep) {
            servos.set_angle(test_channels[i],  20.0f); delay(400);
            servos.set_angle(test_channels[i], -20.0f); delay(400);
            servos.set_angle(test_channels[i],   0.0f); delay(200);
        }
    }
    Serial.println("[TEST] Sweep done.");
}

// ─── setup() ──────────────────────────────────────────────────────────────────
void setup() {
    Serial.begin(115200);
    Serial.println("[Optimus] Booting...");

    state_mutex = xSemaphoreCreateMutex();

    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ_HZ);
    scan_i2c();

#if USE_IMU
    if (!imu.begin(MPU6050_ADDR, I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ_HZ)) {
        Serial.println("[IMU] MPU6050 not found! Check wiring.");
    } else {
        Serial.println("[IMU] MPU6050 OK");
        imu.calibrate(200);
    }
#else
    Serial.println("[IMU] Disabled (USE_IMU=0) — running open-loop CPG");
#endif

    if (!servos.begin(PCA9685_ADDR, PCA9685_FREQ_HZ)) {
        Serial.println("[SERVO] PCA9685 not found!");
    } else {
        Serial.println("[SERVO] PCA9685 OK");
        servos.set_all_neutral();
        delay(500);

#if ENABLE_GAIT
        Serial.println("[SERVO] Ramping to standby pose...");
        for (int step = 0; step <= 20; ++step) {
            float t = (float)step / 20.0f;
            JointAngles standby = {
                .hip_roll   = 0.0f,
                .hip_pitch  = 0.0f,
                .knee       = CPG_KNEE_STANDING_OFFSET_DEG * DEG2RAD * t,
                .ankle_roll = 0.0f
            };
            write_leg(standby, 0);
            write_leg(standby, 1);
            delay(50);
        }
        Serial.println("[SERVO] Standby pose reached.");
        delay(500);
#else
        Serial.println("[SERVO] Halt pose + T-pose arms (ENABLE_GAIT=0)");
        JointAngles halt = { .hip_roll = 0.0f, .hip_pitch = 0.0f,
                             .knee = 0.0f,     .ankle_roll = 0.0f };
        write_leg(halt, 0);
        write_leg(halt, 1);
        write_arms_tpose();
#endif

#if SERVO_SWEEP_TEST
        servo_sweep_test();
#endif
    }

    CPGParams cpg_params = {
        .period_s               = CPG_PERIOD_S,
        .hip_roll_amp           = CPG_HIP_ROLL_AMP_DEG    * DEG2RAD,
        .hip_pitch_amp          = CPG_HIP_PITCH_AMP_DEG   * DEG2RAD,
        .knee_amp               = CPG_KNEE_AMP_DEG        * DEG2RAD,
        .ankle_roll_amp         = CPG_ANKLE_ROLL_AMP_DEG  * DEG2RAD,
        .hip_roll_phase_offset  = CPG_PHASE_HIP_ROLL_DEG  * DEG2RAD,
        .knee_phase_offset      = CPG_PHASE_KNEE_OFFSET_DEG * DEG2RAD,
        .hip_pitch_offset_L     = CPG_HIP_PITCH_TRIM_L_DEG * DEG2RAD,
        .hip_pitch_offset_R     = CPG_HIP_PITCH_TRIM_R_DEG * DEG2RAD,
        .hip_roll_offset_L      = CPG_HIP_ROLL_TRIM_L,
        .hip_roll_offset_R      = CPG_HIP_ROLL_TRIM_R,
        .ankle_roll_offset_L    = 0.0f,
        .ankle_roll_offset_R    = 0.0f,
        .knee_offset_L          = CPG_KNEE_STANDING_OFFSET_DEG * DEG2RAD,
        .knee_offset_R          = CPG_KNEE_STANDING_OFFSET_DEG * DEG2RAD,
    };
    cpg.init(cpg_params);

    stabilizer.init(STAB_PITCH_GAIN, STAB_ROLL_GAIN,
                    STAB_PITCH_THRESHOLD_DEG * DEG2RAD,
                    STAB_ROLL_THRESHOLD_DEG  * DEG2RAD);

    telemetry.set_param_callback(on_param);
    if (!telemetry.begin(WIFI_SSID, WIFI_PASSWORD, WS_PORT)) {
        Serial.println("[WIFI] Connection failed — running without telemetry");
    } else {
        Serial.print("[WIFI] Connected, WS port ");
        Serial.println(WS_PORT);
    }

    // ── FreeRTOS tasks ────────────────────────────────────────────────────────
    // All pinned to core 0 (ESP32-C3 is single-core).
    // Exactly one experiment block is active; the others compile to nothing.
#if USE_IMU
    xTaskCreatePinnedToCore(task_imu, "imu", TASK_IMU_STACK, nullptr, TASK_IMU_PRIORITY, nullptr, 0);
#endif

#if ENABLE_GAIT
    xTaskCreatePinnedToCore(task_cpg, "cpg", TASK_CPG_STACK, nullptr, TASK_CPG_PRIORITY, nullptr, 0);
#elif ENABLE_WALK
    xTaskCreatePinnedToCore(task_walk, "walk", TASK_CPG_STACK, nullptr, TASK_CPG_PRIORITY, nullptr, 0);
    Serial.println("[Optimus] Walk task started (ENABLE_WALK=1)");
#elif ENABLE_BALANCE
    xTaskCreatePinnedToCore(task_balance, "balance", TASK_CPG_STACK, nullptr, TASK_CPG_PRIORITY, nullptr, 0);
    Serial.println("[Optimus] Balance task started (ENABLE_BALANCE=1)");
#elif ENABLE_SINGLE_LEG_BALANCE
    xTaskCreatePinnedToCore(task_single_leg_balance, "slb", TASK_CPG_STACK, nullptr, TASK_CPG_PRIORITY, nullptr, 0);
    xTaskCreatePinnedToCore(task_serial_console,     "con", 3096,           nullptr, 0,                 nullptr, 0);
    Serial.println("[Optimus] Single-leg balance + serial console started (ENABLE_SINGLE_LEG_BALANCE=1)");
#elif ENABLE_IK_SWAY
    xTaskCreatePinnedToCore(task_ik_sway, "ik_sway", TASK_CPG_STACK, nullptr, TASK_CPG_PRIORITY, nullptr, 0);
    Serial.println("[Optimus] IK sagittal sway started (ENABLE_IK_SWAY=1)");
#else
    Serial.println("[Optimus] Passive hold — servos at halt/T-pose");
#endif

    xTaskCreatePinnedToCore(task_telemetry, "telemetry", TASK_TELEMETRY_STACK, nullptr, TASK_TELEMETRY_PRIORITY, nullptr, 0);

    Serial.println("[Optimus] Tasks started.");
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}
