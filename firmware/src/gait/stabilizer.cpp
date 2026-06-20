#include "stabilizer.h"
#include "../../include/config.h"
#include <math.h>

static constexpr float DEG2RAD = 0.017453293f;

Stabilizer::Stabilizer()
    : pitch_gain_(STAB_PITCH_GAIN),
      roll_gain_(STAB_ROLL_GAIN),
      pitch_threshold_(STAB_PITCH_THRESHOLD_DEG * DEG2RAD),
      roll_threshold_(STAB_ROLL_THRESHOLD_DEG  * DEG2RAD) {}

void Stabilizer::init(float pitch_gain, float roll_gain,
                       float pitch_threshold, float roll_threshold) {
    pitch_gain_      = pitch_gain;
    roll_gain_       = roll_gain;
    pitch_threshold_ = pitch_threshold;
    roll_threshold_  = roll_threshold;
}

Stabilizer::Correction Stabilizer::compute(const IMUEstimate& imu) {
    Correction c = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

    // ── Pitch correction ──────────────────────────────────────────────────────
    if (fabsf(imu.pitch_rad) > pitch_threshold_) {
        float correction = -pitch_gain_ * imu.pitch_rad;
        c.hip_pitch_L = correction;
        c.hip_pitch_R = correction;
    }

    // ── Roll correction ───────────────────────────────────────────────────────
    if (fabsf(imu.roll_rad) > roll_threshold_) {
        float correction = -roll_gain_ * imu.roll_rad;
        c.ankle_roll_L =  correction;
        c.ankle_roll_R = -correction;
    }

    // ── Yaw damping (hip yaw servo, ch 10) ───────────────────────────────────
    // Proportional to yaw rate — keeps the upper body in the same azimuthal plane.
    // No threshold: even slow drift should be actively damped.
    c.hip_yaw = -BALANCE_YAW_RATE_GAIN * imu.yaw_rate_rad_s;

    return c;
}
