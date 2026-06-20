// ─── Experiment: T-pose bilateral balance (ENABLE_BALANCE=1) ─────────────────
// Holds both legs at halt pose; applies IMU-derived pitch/roll/yaw corrections
// via hip pitch, hip roll, ankles, and arms.  No locomotion — pure stabilisation.
// Enabled by: #define ENABLE_BALANCE 1  in config.h
#include "../../include/config.h"
#if ENABLE_BALANCE

#include "shared.h"

void task_balance(void* /*arg*/) {
    const TickType_t period = pdMS_TO_TICKS(1000 / TASK_CPG_HZ);
    TickType_t last_wake = xTaskGetTickCount();

    for (;;) {
        IMUEstimate imu_est;
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        imu_est = {g_imu_estimate.pitch_rad, g_imu_estimate.roll_rad, g_imu_estimate.yaw_rate_rad_s};
        xSemaphoreGive(state_mutex);

        if (fabsf(imu_est.pitch_rad) > SAFETY_TILT_LIMIT_DEG * DEG2RAD ||
            fabsf(imu_est.roll_rad)  > SAFETY_TILT_LIMIT_DEG * DEG2RAD) {
            servos.all_off();
            Serial.println("[SAFETY] Tilt limit exceeded — servos halted");
            vTaskSuspend(nullptr);
        }

        JointAngles left  = {0.0f, 0.0f, 0.0f, 0.0f};
        JointAngles right = {0.0f, 0.0f, 0.0f, 0.0f};

        const uint16_t arm_center = (ARM_SERVO_PWM_MIN + ARM_SERVO_PWM_MAX) / 2;
        const float    arm_range  = (ARM_SERVO_PWM_MAX - ARM_SERVO_PWM_MIN) / 2.0f;

        // Pitch: both hips + both arms swing together to shift upper-body CoM
        float arm_fb_deg = 0.0f;
        if (fabsf(imu_est.pitch_rad) > STAB_PITCH_THRESHOLD_DEG * DEG2RAD) {
            float c = -BALANCE_PITCH_GAIN * imu_est.pitch_rad;
            left.hip_pitch  = c;
            right.hip_pitch = c;
            arm_fb_deg = -BALANCE_ARM_PITCH_GAIN * imu_est.pitch_rad * RAD2DEG;
            if (arm_fb_deg >  60.0f) arm_fb_deg =  60.0f;
            if (arm_fb_deg < -60.0f) arm_fb_deg = -60.0f;
        }
        servos.set_pulse_us(SERVO_R_SHOULDER_FB,
            (uint16_t)(arm_center + arm_fb_deg / 90.0f * arm_range));
        servos.set_pulse_us(SERVO_L_SHOULDER_FB,
            (uint16_t)(arm_center - arm_fb_deg / 90.0f * arm_range));

        // Roll: hip roll + ankles + arms lateral
        float arm_lat_deg = 0.0f;
        if (fabsf(imu_est.roll_rad) > STAB_ROLL_THRESHOLD_DEG * DEG2RAD) {
            float c_hip   = -BALANCE_ROLL_GAIN       * imu_est.roll_rad;
            float c_ankle = -BALANCE_ANKLE_ROLL_GAIN * imu_est.roll_rad;
            left.hip_roll    =  c_hip;
            right.hip_roll   = -c_hip;
            left.ankle_roll  =  c_ankle;
            right.ankle_roll = -c_ankle;
            arm_lat_deg = -BALANCE_ARM_ROLL_GAIN * imu_est.roll_rad * RAD2DEG;
            if (arm_lat_deg >  45.0f) arm_lat_deg =  45.0f;
            if (arm_lat_deg < -45.0f) arm_lat_deg = -45.0f;
        }
        servos.set_pulse_us(SERVO_R_SHOULDER_LAT,
            (uint16_t)(arm_center + arm_lat_deg / 90.0f * arm_range));
        servos.set_pulse_us(SERVO_L_SHOULDER_LAT,
            (uint16_t)(arm_center - arm_lat_deg / 90.0f * arm_range));

        write_leg(left,  0);
        write_leg(right, 1);

        // Yaw: damp rotation via hip yaw servo (ch 10)
        float yaw_deg = -BALANCE_YAW_RATE_GAIN * imu_est.yaw_rate_rad_s * RAD2DEG;
        if (yaw_deg >  BALANCE_YAW_MAX_DEG) yaw_deg =  BALANCE_YAW_MAX_DEG;
        if (yaw_deg < -BALANCE_YAW_MAX_DEG) yaw_deg = -BALANCE_YAW_MAX_DEG;
        servos.set_pulse_us(SERVO_HIP_YAW,
            (uint16_t)(arm_center + yaw_deg / 90.0f * arm_range));

        vTaskDelayUntil(&last_wake, period);
    }
}

#endif  // ENABLE_BALANCE
