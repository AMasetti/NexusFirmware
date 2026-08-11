/*
 * firmware-rl — RL inference client for Optimus
 *
 * The ESP32 is a thin hardware bridge:
 *   - Reads IMU at 100 Hz, streams pitch/roll/yaw_rate + raw joints to PC via WebSocket
 *   - Receives absolute joint angles from PC (50 Hz) and writes to PCA9685
 *   - Applies per-joint soft limits and a safety tilt cutoff
 *   - Freezes servos if PC stops sending within SERVO_TIMEOUT_MS
 *
 * All CPG and RL inference runs on the PC (see optimus/tools/rl_runner.py).
 *
 * WebSocket protocol (port 81):
 *
 *   Outgoing JSON at 50 Hz:
 *   { "t": <ms>, "imu": { "pitch": <rad>, "roll": <rad>, "yaw_rate": <rad/s>,
 *                         "gx": <rad/s>, "gy": <rad/s>, "gz": <rad/s> },
 *     "joints": { "l_hip_roll": <rad>, ... } }
 *
 *   Incoming JSON from PC:
 *   { "cmd": "set_joints", "angles": {
 *       "l_hip_roll": <rad>, "l_hip_pitch": <rad>, "l_knee": <rad>, "l_ankle_roll": <rad>,
 *       "r_hip_roll": <rad>, "r_hip_pitch": <rad>, "r_knee": <rad>, "r_ankle_roll": <rad>,
 *       "l_shoulder_fb": <rad>, "l_shoulder_lat": <rad>, "l_forearm_lat": <rad>,
 *       "r_shoulder_fb": <rad>, "r_shoulder_lat": <rad>, "r_forearm_lat": <rad>,
 *       "hip_yaw": <rad> } }
 *   { "cmd": "halt" }            — freeze servos at current pose
 *   { "cmd": "calibrate_imu" }   — re-run IMU calibration
 */

#include <Arduino.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <WebSocketsServer.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

#include "include/config.h"

// Pull in the same IMU and servo drivers as firmware/
#include "src/imu/mpu6050.h"
#include "src/servo/pca9685.h"

// ─── Globals ──────────────────────────────────────────────────────────────────

static MPU6050         imu;
static PCA9685         servos;
static WebSocketsServer ws(WS_PORT);
static SemaphoreHandle_t state_mutex;

static constexpr float DEG2RAD = M_PI / 180.0f;
static constexpr float RAD2DEG = 180.0f / M_PI;

struct IMUState {
    float pitch    = 0.0f;   // rad, complementary filter
    float roll     = 0.0f;   // rad
    float yaw_rate = 0.0f;   // rad/s
    float gx       = 0.0f;   // raw gyro rad/s
    float gy       = 0.0f;
    float gz       = 0.0f;
};

// 15 DOF — order matches ACTUATOR_NAMES in optimus_cpg_env.py
struct JointAngles {
    float l_hip_roll    = 0.0f;
    float l_hip_pitch   = 0.0f;
    float l_knee        = 0.0f;
    float l_ankle_roll  = 0.0f;
    float r_hip_roll    = 0.0f;
    float r_hip_pitch   = 0.0f;
    float r_knee        = 0.0f;
    float r_ankle_roll  = 0.0f;
    float l_shoulder_fb  = 0.0f;
    float r_shoulder_fb  = 0.0f;
    float l_shoulder_lat = 0.0f;
    float r_shoulder_lat = 0.0f;
    float l_forearm_lat  = 0.0f;
    float r_forearm_lat  = 0.0f;
    float hip_yaw        = 0.0f;
};

static volatile IMUState    g_imu;
static volatile JointAngles g_joints;
static volatile uint32_t    g_last_cmd_ms = 0;
static volatile bool        g_halted      = false;

// ─── Servo helpers ────────────────────────────────────────────────────────────

static inline float clampf(float v, float lo, float hi) {
    return v < lo ? lo : (v > hi ? hi : v);
}

// Write a leg servo: angle in radians, converted to physical degrees via cal table.
static void write_leg_servo(uint8_t ch, float angle_rad, float dir, float offset_deg,
                             float min_deg, float max_deg) {
    float deg = 90.0f + angle_rad * RAD2DEG * dir + offset_deg;
    deg = clampf(deg, 90.0f + min_deg, 90.0f + max_deg);
    servos.set_angle(ch, deg - 90.0f);   // PCA9685 driver expects signed degrees from center
}

// Write an arm servo (Futaba S3003): angle in radians → pulse in µs.
static void write_arm_servo(uint8_t ch, float angle_rad, float min_deg, float max_deg) {
    float deg = clampf(angle_rad * RAD2DEG, min_deg, max_deg);
    float t   = (deg - (-90.0f)) / 180.0f;
    uint16_t us = (uint16_t)(ARM_SERVO_PWM_MIN + t * (ARM_SERVO_PWM_MAX - ARM_SERVO_PWM_MIN));
    servos.set_pulse_us(ch, us);
}

static void apply_joints(const JointAngles& q) {
    // Legs (MG995)
    write_leg_servo(SERVO_L_HIP_ROLL,   q.l_hip_roll,   SERVO_DIR_L_HIP_ROLL,   SERVO_OFFSET_DEG_L_HIP_ROLL,   JOINT_HIP_ROLL_MIN_DEG,   JOINT_HIP_ROLL_MAX_DEG);
    write_leg_servo(SERVO_L_HIP_PITCH,  q.l_hip_pitch,  SERVO_DIR_L_HIP_PITCH,  SERVO_OFFSET_DEG_L_HIP_PITCH,  JOINT_HIP_PITCH_MIN_DEG,  JOINT_HIP_PITCH_MAX_DEG);
    write_leg_servo(SERVO_L_KNEE,       q.l_knee,       SERVO_DIR_L_KNEE,       SERVO_OFFSET_DEG_L_KNEE,       JOINT_KNEE_MIN_DEG,       JOINT_KNEE_MAX_DEG);
    write_leg_servo(SERVO_L_ANKLE_ROLL, q.l_ankle_roll, SERVO_DIR_L_ANKLE_ROLL, SERVO_OFFSET_DEG_L_ANKLE_ROLL, JOINT_ANKLE_ROLL_MIN_DEG, JOINT_ANKLE_ROLL_MAX_DEG);
    write_leg_servo(SERVO_R_HIP_ROLL,   q.r_hip_roll,   SERVO_DIR_R_HIP_ROLL,   SERVO_OFFSET_DEG_R_HIP_ROLL,   JOINT_HIP_ROLL_MIN_DEG,   JOINT_HIP_ROLL_MAX_DEG);
    write_leg_servo(SERVO_R_HIP_PITCH,  q.r_hip_pitch,  SERVO_DIR_R_HIP_PITCH,  SERVO_OFFSET_DEG_R_HIP_PITCH,  JOINT_HIP_PITCH_MIN_DEG,  JOINT_HIP_PITCH_MAX_DEG);
    write_leg_servo(SERVO_R_KNEE,       q.r_knee,       SERVO_DIR_R_KNEE,       SERVO_OFFSET_DEG_R_KNEE,       JOINT_KNEE_MIN_DEG,       JOINT_KNEE_MAX_DEG);
    write_leg_servo(SERVO_R_ANKLE_ROLL, q.r_ankle_roll, SERVO_DIR_R_ANKLE_ROLL, SERVO_OFFSET_DEG_R_ANKLE_ROLL, JOINT_ANKLE_ROLL_MIN_DEG, JOINT_ANKLE_ROLL_MAX_DEG);

    // Arms (Futaba S3003)
    write_arm_servo(SERVO_L_SHOULDER_FB,  q.l_shoulder_fb,  JOINT_SHOULDER_FB_MIN_DEG,  JOINT_SHOULDER_FB_MAX_DEG);
    write_arm_servo(SERVO_R_SHOULDER_FB,  q.r_shoulder_fb,  JOINT_SHOULDER_FB_MIN_DEG,  JOINT_SHOULDER_FB_MAX_DEG);
    write_arm_servo(SERVO_L_SHOULDER_LAT, q.l_shoulder_lat, JOINT_SHOULDER_LAT_MIN_DEG, JOINT_SHOULDER_LAT_MAX_DEG);
    write_arm_servo(SERVO_R_SHOULDER_LAT, q.r_shoulder_lat, JOINT_SHOULDER_LAT_MIN_DEG, JOINT_SHOULDER_LAT_MAX_DEG);
    write_arm_servo(SERVO_L_FOREARM_LAT,  q.l_forearm_lat,  JOINT_FOREARM_LAT_MIN_DEG,  JOINT_FOREARM_LAT_MAX_DEG);
    write_arm_servo(SERVO_R_FOREARM_LAT,  q.r_forearm_lat,  JOINT_FOREARM_LAT_MIN_DEG,  JOINT_FOREARM_LAT_MAX_DEG);
    write_arm_servo(SERVO_HIP_YAW,        q.hip_yaw,        JOINT_HIP_YAW_MIN_DEG,      JOINT_HIP_YAW_MAX_DEG);
}

// ─── WebSocket ────────────────────────────────────────────────────────────────

static void on_ws_event(uint8_t num, WStype_t type, uint8_t* payload, size_t len) {
    if (type != WStype_TEXT) return;

    StaticJsonDocument<512> doc;
    if (deserializeJson(doc, payload, len) != DeserializationError::Ok) return;

    const char* cmd = doc["cmd"] | "";

    if (strcmp(cmd, "set_joints") == 0) {
        JsonObject a = doc["angles"];
        if (a.isNull()) return;

        JointAngles q;
        q.l_hip_roll    = a["l_hip_roll"]    | 0.0f;
        q.l_hip_pitch   = a["l_hip_pitch"]   | 0.0f;
        q.l_knee        = a["l_knee"]        | 0.0f;
        q.l_ankle_roll  = a["l_ankle_roll"]  | 0.0f;
        q.r_hip_roll    = a["r_hip_roll"]    | 0.0f;
        q.r_hip_pitch   = a["r_hip_pitch"]   | 0.0f;
        q.r_knee        = a["r_knee"]        | 0.0f;
        q.r_ankle_roll  = a["r_ankle_roll"]  | 0.0f;
        q.l_shoulder_fb  = a["l_shoulder_fb"]  | 0.0f;
        q.r_shoulder_fb  = a["r_shoulder_fb"]  | 0.0f;
        q.l_shoulder_lat = a["l_shoulder_lat"] | 0.0f;
        q.r_shoulder_lat = a["r_shoulder_lat"] | 0.0f;
        q.l_forearm_lat  = a["l_forearm_lat"]  | 0.0f;
        q.r_forearm_lat  = a["r_forearm_lat"]  | 0.0f;
        q.hip_yaw        = a["hip_yaw"]        | 0.0f;

        xSemaphoreTake(state_mutex, portMAX_DELAY);
        memcpy((void*)&g_joints, &q, sizeof(JointAngles));
        g_last_cmd_ms = millis();
        g_halted = false;
        xSemaphoreGive(state_mutex);

    } else if (strcmp(cmd, "halt") == 0) {
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        g_halted = true;
        xSemaphoreGive(state_mutex);
        Serial.println("[RL] Halt command received");

    } else if (strcmp(cmd, "calibrate_imu") == 0) {
        imu.calibrate(200);
        Serial.println("[IMU] Recalibrated");
    }
}

// ─── Task: IMU @ 100 Hz ───────────────────────────────────────────────────────

static void task_imu(void*) {
    const TickType_t period = pdMS_TO_TICKS(1000 / TASK_IMU_HZ);
    const float dt = 1.0f / TASK_IMU_HZ;
    TickType_t wake = xTaskGetTickCount();

    for (;;) {
        if (imu.update(dt)) {
            IMUEstimate est = imu.estimate();
            xSemaphoreTake(state_mutex, portMAX_DELAY);
            g_imu.pitch    = est.pitch;
            g_imu.roll     = est.roll;
            g_imu.yaw_rate = est.yaw_rate;
            MPU6050::RawData raw = imu.raw();
            // Convert raw gyro int16 → rad/s (250 dps range: 131 LSB/°/s)
            g_imu.gx = raw.gx / 131.0f * DEG2RAD;
            g_imu.gy = raw.gy / 131.0f * DEG2RAD;
            g_imu.gz = raw.gz / 131.0f * DEG2RAD;
            xSemaphoreGive(state_mutex);
        }
        vTaskDelayUntil(&wake, period);
    }
}

// ─── Task: Servo output + safety @ 50 Hz ─────────────────────────────────────

static void task_servo(void*) {
    const TickType_t period = pdMS_TO_TICKS(1000 / TASK_SERVO_HZ);
    TickType_t wake = xTaskGetTickCount();
    bool frozen = false;

    for (;;) {
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        IMUState   imu_snap    = g_imu;
        JointAngles joints_snap;
        memcpy(&joints_snap, (const void*)&g_joints, sizeof(JointAngles));
        bool halted          = g_halted;
        uint32_t last_cmd_ms = g_last_cmd_ms;
        xSemaphoreGive(state_mutex);

        float tilt_deg = sqrtf(imu_snap.pitch * imu_snap.pitch +
                               imu_snap.roll  * imu_snap.roll) * RAD2DEG;

        // Safety: tilt cutoff or explicit halt
        if (tilt_deg > SAFETY_TILT_LIMIT_DEG || halted) {
            if (!frozen) {
                Serial.printf("[SAFETY] Tilt %.1f° — freezing servos\n", tilt_deg);
                frozen = true;
            }
            vTaskDelayUntil(&wake, period);
            continue;
        }
        frozen = false;

        // Timeout: PC went silent
        if (millis() - last_cmd_ms > SERVO_TIMEOUT_MS && last_cmd_ms > 0) {
            // Hold last pose silently — PC may be computing a heavy frame
            vTaskDelayUntil(&wake, period);
            continue;
        }

        apply_joints(joints_snap);
        vTaskDelayUntil(&wake, period);
    }
}

// ─── Task: Telemetry @ 50 Hz ─────────────────────────────────────────────────

static void task_telemetry(void*) {
    const TickType_t period = pdMS_TO_TICKS(1000 / TASK_TELEMETRY_HZ);
    TickType_t wake = xTaskGetTickCount();
    char buf[512];

    for (;;) {
        ws.loop();

        xSemaphoreTake(state_mutex, portMAX_DELAY);
        IMUState   imu_snap = g_imu;
        JointAngles j;
        memcpy(&j, (const void*)&g_joints, sizeof(JointAngles));
        xSemaphoreGive(state_mutex);

        snprintf(buf, sizeof(buf),
            "{\"t\":%lu,"
            "\"imu\":{\"pitch\":%.4f,\"roll\":%.4f,\"yaw_rate\":%.4f,"
                     "\"gx\":%.4f,\"gy\":%.4f,\"gz\":%.4f},"
            "\"joints\":{"
                "\"l_hip_roll\":%.4f,\"l_hip_pitch\":%.4f,"
                "\"l_knee\":%.4f,\"l_ankle_roll\":%.4f,"
                "\"r_hip_roll\":%.4f,\"r_hip_pitch\":%.4f,"
                "\"r_knee\":%.4f,\"r_ankle_roll\":%.4f,"
                "\"l_shoulder_fb\":%.4f,\"r_shoulder_fb\":%.4f,"
                "\"l_shoulder_lat\":%.4f,\"r_shoulder_lat\":%.4f,"
                "\"l_forearm_lat\":%.4f,\"r_forearm_lat\":%.4f,"
                "\"hip_yaw\":%.4f}}",
            millis(),
            imu_snap.pitch, imu_snap.roll, imu_snap.yaw_rate,
            imu_snap.gx, imu_snap.gy, imu_snap.gz,
            j.l_hip_roll,   j.l_hip_pitch,
            j.l_knee,       j.l_ankle_roll,
            j.r_hip_roll,   j.r_hip_pitch,
            j.r_knee,       j.r_ankle_roll,
            j.l_shoulder_fb, j.r_shoulder_fb,
            j.l_shoulder_lat, j.r_shoulder_lat,
            j.l_forearm_lat,  j.r_forearm_lat,
            j.hip_yaw);

        ws.broadcastTXT(buf);
        vTaskDelayUntil(&wake, period);
    }
}

// ─── setup() ─────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    Serial.println("[RL] Booting firmware-rl...");

    state_mutex = xSemaphoreCreateMutex();

    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ_HZ);

    if (!imu.begin(MPU6050_ADDR, I2C_SDA_PIN, I2C_SCL_PIN, I2C_FREQ_HZ)) {
        Serial.println("[IMU] MPU6050 not found — check wiring");
    } else {
        Serial.println("[IMU] MPU6050 OK — calibrating...");
        imu.calibrate(200);
        Serial.println("[IMU] Calibration done");
    }

    if (!servos.begin(PCA9685_ADDR, PCA9685_FREQ_HZ)) {
        Serial.println("[SERVO] PCA9685 not found");
    } else {
        Serial.println("[SERVO] PCA9685 OK — moving to halt pose");
        servos.set_all_neutral();
        delay(500);
    }

    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    Serial.print("[WIFI] Connecting");
    while (WiFi.status() != WL_CONNECTED) {
        delay(250);
        Serial.print(".");
    }
    Serial.printf("\n[WIFI] Connected — IP %s\n", WiFi.localIP().toString().c_str());

    MDNS.begin(MDNS_HOSTNAME);
    Serial.printf("[MDNS] Reachable as %s.local\n", MDNS_HOSTNAME);

    ws.begin();
    ws.onEvent(on_ws_event);
    Serial.printf("[WS] WebSocket server on port %d\n", WS_PORT);

    xTaskCreatePinnedToCore(task_imu,       "imu",       TASK_IMU_STACK,       nullptr, TASK_IMU_PRIORITY,       nullptr, 0);
    xTaskCreatePinnedToCore(task_servo,     "servo",     TASK_SERVO_STACK,     nullptr, TASK_SERVO_PRIORITY,     nullptr, 0);
    xTaskCreatePinnedToCore(task_telemetry, "telemetry", TASK_TELEMETRY_STACK, nullptr, TASK_TELEMETRY_PRIORITY, nullptr, 0);

    Serial.println("[RL] Ready — waiting for PC runner");
}

void loop() {
    vTaskDelay(pdMS_TO_TICKS(1000));
}
