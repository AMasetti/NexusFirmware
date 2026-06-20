// ─── Experiment: IK sagittal sway (ENABLE_IK_SWAY=1) ─────────────────────────
// Both feet fixed on ground. Hip oscillates forward/backward in the sagittal
// plane at IK_SWAY_PERIOD_S. IK keeps the foot contact point fixed while
// hip_pitch and knee adapt each cycle.
//
// Coordinate convention (FK from docs/kinematics.md):
//   θ₂_fk  = absolute femur angle from horizontal (-π/2 = pointing straight down)
//   θ₃_fk  = knee angle (0 = straight, positive = bent)
//   hip_pitch_joint = θ₂_fk + π/2  (joint-space: 0 = standing, as used by write_leg)
//
// FK (foot position relative to hip):
//   x = L1·cos(θ₂) + L2·cos(θ₂+θ₃) + L3
//   y = L1·sin(θ₂) + L2·sin(θ₂+θ₃)
//
// IK given foot (x,y) relative to hip:
//   d     = sqrt((x-L3)² + y²)
//   cos_k = (d²-L1²-L2²)/(2·L1·L2)
//   θ₃    = atan2(sqrt(1-cos_k²), cos_k)
//   θ₂    = atan2(y, x-L3) - atan2(L2·sin(θ₃), L1+L2·cos(θ₃))
#include "../../include/config.h"
#if ENABLE_IK_SWAY

#include <math.h>
#include "shared.h"

// Returns θ₂_fk and θ₃_fk (FK absolute angles). Clamps at workspace boundary.
static void sagittal_ik(float x_rel, float y_rel,
                         float& theta2_fk, float& theta3_fk)
{
    const float L1 = L1_MM, L2 = L2_MM, L3 = L3_MM;
    float dx   = x_rel - L3;
    float d2   = dx * dx + y_rel * y_rel;
    float cos_k = (d2 - L1*L1 - L2*L2) / (2.0f * L1 * L2);
    if (cos_k < -1.0f) cos_k = -1.0f;   // clamp to workspace boundary, no snap
    if (cos_k >  1.0f) cos_k =  1.0f;
    theta3_fk = atan2f(sqrtf(1.0f - cos_k * cos_k), cos_k);
    theta2_fk = atan2f(y_rel, dx) - atan2f(L2 * sinf(theta3_fk), L1 + L2 * cosf(theta3_fk));
}

void task_ik_sway(void* /*arg*/) {
    const TickType_t period = pdMS_TO_TICKS(1000 / TASK_CPG_HZ);
    TickType_t last_wake    = xTaskGetTickCount();

    // ── Neutral foot position via FK at symmetric squat pose ─────────────────
    // hip_pitch dir=-1 inverts servo travel vs knee dir=+1.
    // To get equal visual bend: hip_pitch_joint = knee = IK_SWAY_KNEE_DEG.
    // θ₂_fk = hip_pitch_joint - π/2
    const float theta3_neutral = IK_SWAY_KNEE_DEG * DEG2RAD;
    const float theta2_neutral = IK_SWAY_KNEE_DEG * DEG2RAD - (float)M_PI / 2.0f;

    const float foot_x0 = L1_MM * cosf(theta2_neutral)
                        + L2_MM * cosf(theta2_neutral + theta3_neutral)
                        + L3_MM;
    const float foot_y0 = L1_MM * sinf(theta2_neutral)
                        + L2_MM * sinf(theta2_neutral + theta3_neutral);

    Serial.printf("[IK_SWAY] neutral foot x=%.1fmm y=%.1fmm  knee=%.1fdeg  amp=%.1fmm  T=%.1fs\n",
                  foot_x0, foot_y0, IK_SWAY_KNEE_DEG, IK_SWAY_AMP_MM, IK_SWAY_PERIOD_S);

    const float slew  = IK_SWAY_SLEW_MM_PER_CYCLE;
    const float amp   = IK_SWAY_AMP_MM;
    float current_mm  = 0.0f;  // current hip offset, slewed toward target

    for (;;) {
#if USE_IMU
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
#endif

        // Read joystick target and slew toward it at max IK_SWAY_SLEW_MM_PER_CYCLE.
        float target_mm;
        xSemaphoreTake(state_mutex, portMAX_DELAY);
        target_mm = g_sway_target_mm;
        xSemaphoreGive(state_mutex);

        float err = target_mm - current_mm;
        if      (err >  slew) err =  slew;
        else if (err < -slew) err = -slew;
        current_mm += err;
        if (current_mm >  amp) current_mm =  amp;
        if (current_mm < -amp) current_mm = -amp;

        float x_rel = foot_x0 - current_mm;
        float y_rel = foot_y0;

        float theta2_fk = theta2_neutral;
        float theta3_fk = theta3_neutral;
        sagittal_ik(x_rel, y_rel, theta2_fk, theta3_fk);  // clamps at boundary

        float hp_rad = theta2_fk + (float)M_PI / 2.0f;
        float k_rad  = theta3_fk;

        JointAngles left  = { .hip_roll = 0.0f, .hip_pitch = hp_rad,
                               .knee = k_rad,    .ankle_roll = 0.0f };
        JointAngles right = { .hip_roll = 0.0f, .hip_pitch = hp_rad,
                               .knee = k_rad,    .ankle_roll = 0.0f };

        xSemaphoreTake(state_mutex, portMAX_DELAY);
        memcpy((void*)&g_leg_angles.left,  &left,  sizeof(JointAngles));
        memcpy((void*)&g_leg_angles.right, &right, sizeof(JointAngles));
        xSemaphoreGive(state_mutex);

        write_leg(left,  0);
        write_leg(right, 1);
        write_arms_tpose();

        vTaskDelayUntil(&last_wake, period);
    }
}

#endif  // ENABLE_IK_SWAY
