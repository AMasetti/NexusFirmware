// ─── Experiment: Single-leg balance, right stance (ENABLE_SINGLE_LEG_BALANCE=1) ─
// Left leg rises to a fixed raised pose; right hip + ankle + arms are driven by
// EMA-smoothed IMU corrections every cycle.
//
// Sign conventions (same as task_balance):
//   pitch error (lean forward) → right hip_pitch -= gain × pitch
//   roll  error (lean right)   → right hip_roll  -= gain × roll  (dir=-1 → CoM left)
//                                right ankle_roll -= gain × roll
//   arms:  shoulder FB  both dirs same sign  (shift upper CoM sagittally)
//          shoulder lat opposite signs L/R   (shift upper CoM laterally)
//
// Runtime tuning via serial console (type 'help' after connecting).
// Enabled by: #define ENABLE_SINGLE_LEG_BALANCE 1  in config.h
#include "../../include/config.h"
#if ENABLE_SINGLE_LEG_BALANCE

#include "shared.h"
#include <string.h>

// ─── Runtime-adjustable parameters ───────────────────────────────────────────
// Written only from task_serial_console; read every cycle by task_single_leg_balance.
// On single-core ESP32-C3, volatile float reads/writes are effectively atomic.
struct SlbParams {
    volatile float   lean_deg;           // DC stance lean [°]
    volatile float   pitch_gain;         // hip pitch  [rad / rad pitch error]
    volatile float   roll_gain;          // hip roll   [rad / rad roll error]
    volatile float   ankle_gain;         // ankle roll [rad / rad roll error]
    volatile float   arm_pitch_deg;      // shoulder FB  [deg / rad pitch error]
    volatile float   arm_roll_deg;       // shoulder lat [deg / rad roll error]
    volatile float   smooth_alpha;       // EMA coefficient 0..1
    volatile float   pitch_trim_rad;     // static IMU pitch bias to subtract
    volatile float   roll_trim_rad;      // static IMU roll  bias to subtract
    volatile float   manual_pitch_rad;   // operator nudge — shifts pitch setpoint
    volatile float   manual_roll_rad;    // operator nudge — shifts roll  setpoint
    volatile uint8_t log_hz;             // log rate: 0=off, else N Hz (max 100)
};

static SlbParams g_slb = {
    .lean_deg         = SLB_STANCE_LEAN_DEG,
    .pitch_gain       = SLB_PITCH_GAIN,
    .roll_gain        = SLB_ROLL_GAIN,
    .ankle_gain       = SLB_ANKLE_GAIN,
    .arm_pitch_deg    = SLB_ARM_PITCH_DEG,
    .arm_roll_deg     = SLB_ARM_ROLL_DEG,
    .smooth_alpha     = SLB_SMOOTH_ALPHA,
    .pitch_trim_rad   = 0.0f,
    .roll_trim_rad    = 0.0f,
    .manual_pitch_rad = 0.0f,
    .manual_roll_rad  = 0.0f,
    .log_hz           = 1,
};

// ─── Serial console ───────────────────────────────────────────────────────────

static void slb_print_help() {
    Serial.println("┌─ SLB serial console ────────────────────────────────────┐");
    Serial.println("│ s / status          IMU + param snapshot                │");
    Serial.println("│ log <hz>            log rate: 0=off 1 10 100            │");
    Serial.println("│                                                          │");
    Serial.println("│ lean  <deg>         stance lean angle [°]               │");
    Serial.println("│ pg    <val>         pitch gain  [rad/rad]               │");
    Serial.println("│ rg    <val>         roll gain   [rad/rad]               │");
    Serial.println("│ ag    <val>         ankle gain  [rad/rad]               │");
    Serial.println("│ afb   <val>         arm FB gain [deg/rad]               │");
    Serial.println("│ alat  <val>         arm lat gain [deg/rad]              │");
    Serial.println("│ alpha <0..1>        EMA smoothing (~40ms at 0.25)       │");
    Serial.println("│                                                          │");
    Serial.println("│ ptr   <deg>         pitch trim — removes IMU bias       │");
    Serial.println("│ rtr   <deg>         roll trim  — removes IMU bias       │");
    Serial.println("│ mp    <deg>         manual pitch nudge (+ = fwd lean)   │");
    Serial.println("│ mr    <deg>         manual roll nudge  (+ = right lean) │");
    Serial.println("│                                                          │");
    Serial.println("│ cal                 recalibrate IMU (hold still ~2 s)   │");
    Serial.println("│ stop                cut servo power                     │");
    Serial.println("└─────────────────────────────────────────────────────────┘");
}

static void handle_console_cmd(const char* line) {
    char  cmd[16];
    float val = 0.0f;
    int   n   = sscanf(line, "%15s %f", cmd, &val);

    if (strcmp(cmd, "help") == 0 || strcmp(cmd, "?") == 0) {
        slb_print_help();

    } else if (strcmp(cmd, "status") == 0 || strcmp(cmd, "s") == 0) {
        IMUEstimate e;
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        e = {g_imu_estimate.pitch_rad, g_imu_estimate.roll_rad, g_imu_estimate.yaw_rate_rad_s};
        xSemaphoreGive(state_mutex);
        Serial.printf("  IMU    p=%+.4f r=%+.4f yr=%+.4f\n", e.pitch_rad, e.roll_rad, e.yaw_rate_rad_s);
        Serial.printf("  lean=%.1f°  pg=%.2f  rg=%.2f  ag=%.2f\n",
            (float)g_slb.lean_deg, (float)g_slb.pitch_gain,
            (float)g_slb.roll_gain, (float)g_slb.ankle_gain);
        Serial.printf("  afb=%.1f  alat=%.1f  alpha=%.3f  log=%dHz\n",
            (float)g_slb.arm_pitch_deg, (float)g_slb.arm_roll_deg,
            (float)g_slb.smooth_alpha,  (int)g_slb.log_hz);
        Serial.printf("  trim   pt=%+.2f°  rt=%+.2f°\n",
            (float)g_slb.pitch_trim_rad * RAD2DEG,
            (float)g_slb.roll_trim_rad  * RAD2DEG);
        Serial.printf("  nudge  mp=%+.2f°  mr=%+.2f°\n",
            (float)g_slb.manual_pitch_rad * RAD2DEG,
            (float)g_slb.manual_roll_rad  * RAD2DEG);

    } else if (strcmp(cmd, "cal") == 0) {
        Serial.println("[CON] Calibrating IMU — hold robot still...");
        imu.calibrate(200);
        Serial.println("[CON] Done.");

    } else if (strcmp(cmd, "stop") == 0) {
        servos.all_off();
        Serial.println("[CON] Servos cut.");

    } else if (n == 2) {
        if      (strcmp(cmd, "log")   == 0) g_slb.log_hz           = (uint8_t)val;
        else if (strcmp(cmd, "lean")  == 0) g_slb.lean_deg          = val;
        else if (strcmp(cmd, "pg")    == 0) g_slb.pitch_gain        = val;
        else if (strcmp(cmd, "rg")    == 0) g_slb.roll_gain         = val;
        else if (strcmp(cmd, "ag")    == 0) g_slb.ankle_gain        = val;
        else if (strcmp(cmd, "afb")   == 0) g_slb.arm_pitch_deg     = val;
        else if (strcmp(cmd, "alat")  == 0) g_slb.arm_roll_deg      = val;
        else if (strcmp(cmd, "alpha") == 0) g_slb.smooth_alpha      = val;
        else if (strcmp(cmd, "ptr")   == 0) g_slb.pitch_trim_rad    = val * DEG2RAD;
        else if (strcmp(cmd, "rtr")   == 0) g_slb.roll_trim_rad     = val * DEG2RAD;
        else if (strcmp(cmd, "mp")    == 0) g_slb.manual_pitch_rad  = val * DEG2RAD;
        else if (strcmp(cmd, "mr")    == 0) g_slb.manual_roll_rad   = val * DEG2RAD;
        else { Serial.printf("[CON] Unknown: %s\n", cmd); return; }
        Serial.printf("[CON] %s = %.4f\n", cmd, val);

    } else {
        Serial.printf("[CON] Unknown: '%s'  (type 'help')\n", line);
    }
}

void task_serial_console(void* /*arg*/) {
    vTaskDelay(pdMS_TO_TICKS(1500));   // let boot messages print first
    Serial.println("\n[CON] Ready. Type 'help' for commands.");

    static char buf[64];
    uint8_t     pos = 0;

    for (;;) {
        while (Serial.available()) {
            char c = (char)Serial.read();
            if (c == '\r') continue;
            if (c == '\n') {
                buf[pos] = '\0';
                if (pos > 0) handle_console_cmd(buf);
                pos = 0;
            } else if (pos < sizeof(buf) - 1) {
                buf[pos++] = c;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(20));   // 50 Hz poll
    }
}

// ─── Balance task ─────────────────────────────────────────────────────────────

void task_single_leg_balance(void* /*arg*/) {
    const TickType_t period    = pdMS_TO_TICKS(1000 / TASK_CPG_HZ);
    TickType_t       last_wake = xTaskGetTickCount();

    const uint16_t arm_center = (ARM_SERVO_PWM_MIN + ARM_SERVO_PWM_MAX) / 2;
    const float    arm_range  = (ARM_SERVO_PWM_MAX - ARM_SERVO_PWM_MIN) / 2.0f;

    // Ramp over ~1 s: simultaneously raise left leg and lean CoM over right foot.
    {
        const float init_lean = g_slb.lean_deg * DEG2RAD;
        for (int step = 0; step <= 20; ++step) {
            float frac = (float)step / 20.0f;
            JointAngles left_ramp = {
                .hip_roll   = 0.0f,
                .hip_pitch  = SLB_FLOAT_HIP_PITCH_DEG * DEG2RAD * frac,
                .knee       = SLB_FLOAT_KNEE_DEG       * DEG2RAD * frac,
                .ankle_roll = 0.0f,
            };
            JointAngles right_ramp = {
                .hip_roll   = -init_lean * frac,
                .hip_pitch  = 0.0f,
                .knee       = 0.0f,
                .ankle_roll = +init_lean * frac,
            };
            write_leg(left_ramp,  0);
            write_leg(right_ramp, 1);
            write_arms_tpose();
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }

    const JointAngles left_raised = {
        .hip_roll   = 0.0f,
        .hip_pitch  = SLB_FLOAT_HIP_PITCH_DEG * DEG2RAD,
        .knee       = SLB_FLOAT_KNEE_DEG       * DEG2RAD,
        .ankle_roll = 0.0f,
    };

    float    smooth_pitch = 0.0f;
    float    smooth_roll  = 0.0f;
    uint32_t slb_tick     = 0;

    for (;;) {
        IMUEstimate imu_est;
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        imu_est = {g_imu_estimate.pitch_rad, g_imu_estimate.roll_rad,
                   g_imu_estimate.yaw_rate_rad_s};
        xSemaphoreGive(state_mutex);

        if (fabsf(imu_est.pitch_rad) > SAFETY_TILT_LIMIT_DEG * DEG2RAD ||
            fabsf(imu_est.roll_rad)  > SAFETY_TILT_LIMIT_DEG * DEG2RAD) {
            servos.all_off();
            Serial.println("[SAFETY] Tilt limit exceeded — SLB halted");
            vTaskSuspend(nullptr);
        }

        const float alpha      = g_slb.smooth_alpha;
        const float lean       = g_slb.lean_deg * DEG2RAD;
        const float pitch_trim = g_slb.pitch_trim_rad;
        const float roll_trim  = g_slb.roll_trim_rad;

        smooth_pitch += alpha * (imu_est.pitch_rad - smooth_pitch);
        smooth_roll  += alpha * (imu_est.roll_rad  - smooth_roll);

        // Effective error: trim removes static bias; nudge shifts the equilibrium setpoint
        const float pitch_err = smooth_pitch - pitch_trim - g_slb.manual_pitch_rad;
        const float roll_err  = smooth_roll  - roll_trim  - g_slb.manual_roll_rad;

        JointAngles right = {
            .hip_roll   = -lean - g_slb.roll_gain  * roll_err,
            .hip_pitch  =        -g_slb.pitch_gain * pitch_err,
            .knee       = 0.0f,
            .ankle_roll = +lean - g_slb.ankle_gain * roll_err,
        };

        write_leg(left_raised, 0);
        write_leg(right,       1);

        float arm_fb_deg  = -g_slb.arm_pitch_deg * pitch_err;
        float arm_lat_deg = -g_slb.arm_roll_deg  * roll_err;
        if (arm_fb_deg  >  60.0f) arm_fb_deg  =  60.0f;
        if (arm_fb_deg  < -60.0f) arm_fb_deg  = -60.0f;
        if (arm_lat_deg >  45.0f) arm_lat_deg =  45.0f;
        if (arm_lat_deg < -45.0f) arm_lat_deg = -45.0f;

        servos.set_pulse_us(SERVO_R_SHOULDER_FB,
            (uint16_t)(arm_center + arm_fb_deg  / 90.0f * arm_range));
        servos.set_pulse_us(SERVO_L_SHOULDER_FB,
            (uint16_t)(arm_center - arm_fb_deg  / 90.0f * arm_range));
        servos.set_pulse_us(SERVO_R_SHOULDER_LAT,
            (uint16_t)(arm_center + arm_lat_deg / 90.0f * arm_range));
        servos.set_pulse_us(SERVO_L_SHOULDER_LAT,
            (uint16_t)(arm_center - arm_lat_deg / 90.0f * arm_range));
        servos.set_pulse_us(SERVO_R_FOREARM_LAT, arm_center);
        servos.set_pulse_us(SERVO_L_FOREARM_LAT, arm_center);
        servos.set_pulse_us(SERVO_HIP_YAW,       arm_center);

        const uint8_t lhz = g_slb.log_hz;
        if (lhz > 0) {
            const uint32_t period_ticks = TASK_CPG_HZ / lhz;
            if (++slb_tick >= period_ticks) {
                slb_tick = 0;
                Serial.printf(
                    "[SLB] p=%+.3f r=%+.3f pe=%+.3f re=%+.3f"
                    " | hr=%+.3f hp=%+.3f ar=%+.3f"
                    " | fb=%+.1f lat=%+.1f\n",
                    smooth_pitch, smooth_roll, pitch_err, roll_err,
                    right.hip_roll, right.hip_pitch, right.ankle_roll,
                    arm_fb_deg, arm_lat_deg);
            }
        } else {
            slb_tick = 0;
        }

        vTaskDelayUntil(&last_wake, period);
    }
}

#endif  // ENABLE_SINGLE_LEG_BALANCE
