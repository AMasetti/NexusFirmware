#include "inverse.h"
#include "../../include/config.h"
#include <math.h>

static constexpr float DEG2RAD = 0.01745329252f;

float clamp_angle_rad(float angle_rad, float min_deg, float max_deg) {
    float min_r = min_deg * DEG2RAD;
    float max_r = max_deg * DEG2RAD;
    if (angle_rad < min_r) return min_r;
    if (angle_rad > max_r) return max_r;
    return angle_rad;
}

IKResult ik_solve(Vec3 target, float L1, float L2, float L3,
                  IKElbow elbow) {
    IKResult result;
    result.valid = false;

    float x = target.x;
    float y = target.y;
    float z = target.z;

    // ── Step 1: Hip roll (coronal plane) ──────────────────────────────────────
    // With ankle constraint θ4 = −θ1:
    //   z = (L1+L2)·sin(θ1)  →  θ1 = arcsin(z / (L1+L2))
    float ratio = z / (L1 + L2);
    if (fabsf(ratio) > 1.0f) {
        return result;   // lateral target out of reach
    }
    float theta1 = asinf(ratio);

    // ── Step 2: Effective 2D distance in sagittal plane ───────────────────────
    float dx = x - L3;
    float d_sq = dx * dx + y * y;
    float d    = sqrtf(d_sq);

    // ── Step 3: Knee angle by law of cosines ──────────────────────────────────
    float cos_theta3 = (d_sq - L1 * L1 - L2 * L2) / (2.0f * L1 * L2);

    // Guard: target out of reach
    if (cos_theta3 < -1.0f || cos_theta3 > 1.0f) {
        return result;
    }

    float sin_theta3 = (elbow == IKElbow::UP)
                       ?  sqrtf(1.0f - cos_theta3 * cos_theta3)
                       : -sqrtf(1.0f - cos_theta3 * cos_theta3);

    float theta3 = atan2f(sin_theta3, cos_theta3);

    // ── Step 4: Hip pitch ─────────────────────────────────────────────────────
    float theta2 = atan2f(y, dx)
                 - atan2f(L2 * sin_theta3, L1 + L2 * cos_theta3);

    // ── Step 5: Ankle roll cancels hip roll ───────────────────────────────────
    float theta4 = -theta1;

    // ── Clamp to servo limits ─────────────────────────────────────────────────
    theta1 = clamp_angle_rad(theta1, SERVO_MIN_DEG, SERVO_MAX_DEG);
    theta2 = clamp_angle_rad(theta2, SERVO_MIN_DEG, SERVO_MAX_DEG);
    theta3 = clamp_angle_rad(theta3, SERVO_MIN_DEG, SERVO_MAX_DEG);
    theta4 = clamp_angle_rad(theta4, SERVO_MIN_DEG, SERVO_MAX_DEG);

    result.angles = { theta1, theta2, theta3, theta4 };
    result.valid  = true;
    return result;
}
