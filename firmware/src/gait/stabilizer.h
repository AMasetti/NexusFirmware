#pragma once
#include "cpg.h"
#include "../imu/imu_estimate.h"

// ─── IMU Stabilizer ───────────────────────────────────────────────────────────
// Reads estimated pitch/roll from complementary filter and adjusts CPG offsets
// to counteract body inclination.
//
// Strategy:
//   |pitch| > threshold → adjust hip_pitch_offset (θ2) on both legs equally
//   |roll|  > threshold → adjust ankle_roll_offset (θ4) — critical for lateral balance

class Stabilizer {
public:
    Stabilizer();
    void init(float pitch_gain, float roll_gain,
              float pitch_threshold, float roll_threshold);

    // Compute CPG offset corrections given current IMU estimate.
    // Returns updated offsets that should be passed to CPGOscillator::set_offsets().
    struct Correction {
        float hip_pitch_L, hip_pitch_R;
        float ankle_roll_L, ankle_roll_R;
        float hip_yaw;   // upper-body yaw correction [rad], applied to ch 10
    };

    Correction compute(const IMUEstimate& imu);

private:
    float pitch_gain_;
    float roll_gain_;
    float pitch_threshold_;
    float roll_threshold_;
};
