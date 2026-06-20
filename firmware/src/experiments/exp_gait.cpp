// ─── Experiment: CPG oscillator gait (ENABLE_GAIT=1) ─────────────────────────
// Open-loop sinusoidal CPG with optional IMU stabiliser overlay.
// Enabled by: #define ENABLE_GAIT 1  in config.h
#include "../../include/config.h"
#if ENABLE_GAIT

#include "shared.h"
#include <string.h>

void task_cpg(void* /*arg*/) {
    const TickType_t period = pdMS_TO_TICKS(1000 / TASK_CPG_HZ);
    TickType_t last_wake = xTaskGetTickCount();
    float t = 0.0f;
    const float dt = 1.0f / TASK_CPG_HZ;

    for (;;) {
#if USE_IMU
        IMUEstimate imu_est;
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        imu_est = {g_imu_estimate.pitch_rad, g_imu_estimate.roll_rad};
        xSemaphoreGive(state_mutex);

        if (fabsf(imu_est.pitch_rad) > SAFETY_TILT_LIMIT_DEG * DEG2RAD ||
            fabsf(imu_est.roll_rad)  > SAFETY_TILT_LIMIT_DEG * DEG2RAD) {
            servos.all_off();
            Serial.println("[SAFETY] Tilt limit exceeded — servos halted");
            vTaskSuspend(nullptr);
        }

        auto corr = stabilizer.compute(imu_est);
        cpg.set_offsets(corr.hip_pitch_L, corr.hip_pitch_R,
                        corr.ankle_roll_L, corr.ankle_roll_R);

        {
            float yaw_deg = corr.hip_yaw * RAD2DEG;
            if (yaw_deg >  BALANCE_YAW_MAX_DEG) yaw_deg =  BALANCE_YAW_MAX_DEG;
            if (yaw_deg < -BALANCE_YAW_MAX_DEG) yaw_deg = -BALANCE_YAW_MAX_DEG;
            const uint16_t arm_center = (ARM_SERVO_PWM_MIN + ARM_SERVO_PWM_MAX) / 2;
            const float    arm_range  = (ARM_SERVO_PWM_MAX - ARM_SERVO_PWM_MIN) / 2.0f;
            servos.set_pulse_us(SERVO_HIP_YAW,
                (uint16_t)(arm_center + yaw_deg / 90.0f * arm_range));
        }
#endif  // USE_IMU

        LegAngles angles = cpg.update(t);

        write_leg(angles.left,  0);
        write_leg(angles.right, 1);

        // 4 Hz log: raw CPG angles and physical servo degrees side-by-side
        static uint32_t log_tick = 0;
        if (++log_tick >= 25) {
            log_tick = 0;
            float l_hp_raw = angles.left.hip_pitch  * RAD2DEG;
            float l_k_raw  = angles.left.knee       * RAD2DEG;
            float l_hr_raw = angles.left.hip_roll   * RAD2DEG;
            float r_hp_raw = angles.right.hip_pitch * RAD2DEG;
            float r_k_raw  = angles.right.knee      * RAD2DEG;
            float r_hr_raw = angles.right.hip_roll  * RAD2DEG;

            float l_hp_phy = angles.left.hip_pitch  * RAD2DEG * SERVO_DIR[SERVO_L_HIP_PITCH] + SERVO_OFFSET_DEG[SERVO_L_HIP_PITCH];
            float l_k_phy  = angles.left.knee       * RAD2DEG * SERVO_DIR[SERVO_L_KNEE]       + SERVO_OFFSET_DEG[SERVO_L_KNEE];
            float l_hr_phy = angles.left.hip_roll   * RAD2DEG * SERVO_DIR[SERVO_L_HIP_ROLL]   + SERVO_OFFSET_DEG[SERVO_L_HIP_ROLL];
            float r_hp_phy = angles.right.hip_pitch * RAD2DEG * SERVO_DIR[SERVO_R_HIP_PITCH]  + SERVO_OFFSET_DEG[SERVO_R_HIP_PITCH];
            float r_k_phy  = angles.right.knee      * RAD2DEG * SERVO_DIR[SERVO_R_KNEE]        + SERVO_OFFSET_DEG[SERVO_R_KNEE];
            float r_hr_phy = angles.right.hip_roll  * RAD2DEG * SERVO_DIR[SERVO_R_HIP_ROLL]   + SERVO_OFFSET_DEG[SERVO_R_HIP_ROLL];

            Serial.printf(
                "t=%5.2f | "
                "L hp=%6.1f(%5.1f) k=%6.1f(%5.1f) hr=%6.1f(%5.1f) | "
                "R hp=%6.1f(%5.1f) k=%6.1f(%5.1f) hr=%6.1f(%5.1f)\n",
                t,
                l_hp_raw, l_hp_phy, l_k_raw, l_k_phy, l_hr_raw, l_hr_phy,
                r_hp_raw, r_hp_phy, r_k_raw, r_k_phy, r_hr_raw, r_hr_phy
            );
        }

        xSemaphoreTake(state_mutex, portMAX_DELAY);
        memcpy((void*)&g_leg_angles, &angles, sizeof(LegAngles));
        g_time_s = t;
        xSemaphoreGive(state_mutex);

        t += dt;
        vTaskDelayUntil(&last_wake, period);
    }
}

#endif  // ENABLE_GAIT
