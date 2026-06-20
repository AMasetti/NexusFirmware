// ─── Experiment: IMU-gated walk (ENABLE_WALK=1) ───────────────────────────────
// 4-phase state machine: SHIFT_RIGHT → SWING_LEFT → SHIFT_LEFT → SWING_RIGHT.
// Balance overlay (pitch + roll corrections) runs every cycle atop gait angles.
// Enabled by: #define ENABLE_WALK 1  in config.h
#include "../../include/config.h"
#if ENABLE_WALK

#include "shared.h"

void task_walk(void* /*arg*/) {
    const TickType_t period = pdMS_TO_TICKS(1000 / TASK_CPG_HZ);
    TickType_t last_wake = xTaskGetTickCount();
    const float dt = 1.0f / TASK_CPG_HZ;

    const uint16_t arm_center = (ARM_SERVO_PWM_MIN + ARM_SERVO_PWM_MAX) / 2;
    const float    arm_range  = (ARM_SERVO_PWM_MAX - ARM_SERVO_PWM_MIN) / 2.0f;

    walk_ctrl.init();

    for (;;) {
        IMUEstimate imu_est;
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        imu_est = {g_imu_estimate.pitch_rad, g_imu_estimate.roll_rad,
                   g_imu_estimate.yaw_rate_rad_s};
        xSemaphoreGive(state_mutex);

        if (fabsf(imu_est.pitch_rad) > SAFETY_TILT_LIMIT_DEG * DEG2RAD ||
            fabsf(imu_est.roll_rad)  > SAFETY_TILT_LIMIT_DEG * DEG2RAD) {
            servos.all_off();
            Serial.println("[SAFETY] Tilt limit exceeded — walk halted");
            vTaskSuspend(nullptr);
        }

        WalkController::Output w = walk_ctrl.update(imu_est, dt);

        // EMA smoothing: α=0.25 → ~40 ms lag at 100 Hz
        static float smooth_pitch = 0.0f, smooth_roll = 0.0f;
        smooth_pitch += WALK_BAL_SMOOTH_ALPHA * (imu_est.pitch_rad - smooth_pitch);
        smooth_roll  += WALK_BAL_SMOOTH_ALPHA * (imu_est.roll_rad  - smooth_roll);

        // Roll error relative to walk's commanded lean — near zero when weight
        // transfer is correct; corrections assist rather than fight the lean.
        const float roll_error = smooth_roll - walk_ctrl.target_roll_rad();
        auto        float_leg  = walk_ctrl.float_leg();

        // Pitch: stance leg only — skip float leg to preserve stride length
        if (fabsf(smooth_pitch) > STAB_PITCH_THRESHOLD_DEG * DEG2RAD) {
            float c = -BALANCE_PITCH_GAIN * smooth_pitch;
            if (float_leg != WalkController::FloatLeg::LEFT)  w.left.hip_pitch  += c;
            if (float_leg != WalkController::FloatLeg::RIGHT) w.right.hip_pitch += c;
        }

        // Roll: symmetric L/R; ankle opposes hip to keep sole flat
        if (fabsf(roll_error) > STAB_ROLL_THRESHOLD_DEG * DEG2RAD) {
            float ch = -WALK_BAL_ROLL_GAIN * roll_error;
            w.left.hip_roll    +=  ch;   w.right.hip_roll   += -ch;
            w.left.ankle_roll  += -ch;   w.right.ankle_roll +=  ch;
        }

        // Arm FB: small supplemental pitch trim, capped so it doesn't overwhelm swing
        float arm_fb_corr = -WALK_BAL_ARM_DEG * smooth_pitch;
        if (arm_fb_corr >  15.0f) arm_fb_corr =  15.0f;
        if (arm_fb_corr < -15.0f) arm_fb_corr = -15.0f;
        w.r_shoulder_fb_deg += arm_fb_corr;
        w.l_shoulder_fb_deg += arm_fb_corr;

        const float arm_lat_corr = 0.0f;   // disabled — rest pos already at servo limits

        // 1 Hz log
        static uint32_t walk_log_tick = 0;
        if (++walk_log_tick >= 100) {
            walk_log_tick = 0;
            Serial.printf("[WALK] p=%.3f r=%.3f re=%.3f tr=%.3f | fb=%.1f lat=%.1f fl=%d\n",
                smooth_pitch, smooth_roll, roll_error,
                walk_ctrl.target_roll_rad(), arm_fb_corr, arm_lat_corr, (int)float_leg);
        }

        write_leg(w.left,  0);
        write_leg(w.right, 1);

        // Shoulder FB — swing oscillates around center; R uses +, L uses −
        servos.set_pulse_us(SERVO_R_SHOULDER_FB,
            (uint16_t)(arm_center + w.r_shoulder_fb_deg / 90.0f * arm_range));
        servos.set_pulse_us(SERVO_L_SHOULDER_FB,
            (uint16_t)(arm_center - w.l_shoulder_fb_deg / 90.0f * arm_range));

        servos.set_pulse_us(SERVO_R_SHOULDER_LAT,
            (uint16_t)(arm_center + (WALK_ARM_LAT_REST_DEG + arm_lat_corr) / 90.0f * arm_range));
        servos.set_pulse_us(SERVO_L_SHOULDER_LAT,
            (uint16_t)(arm_center - (WALK_ARM_LAT_REST_DEG + arm_lat_corr) / 90.0f * arm_range));

        const uint16_t r_fore_us = (uint16_t)(arm_center + WALK_ARM_FOREARM_REST_DEG / 90.0f * arm_range);
        const uint16_t l_fore_us = (uint16_t)(arm_center - WALK_ARM_FOREARM_REST_DEG / 90.0f * arm_range);
        servos.set_pulse_us(SERVO_R_FOREARM_LAT, r_fore_us);
        servos.set_pulse_us(SERVO_L_FOREARM_LAT, l_fore_us);

        vTaskDelayUntil(&last_wake, period);
    }
}

#endif  // ENABLE_WALK
