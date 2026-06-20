#pragma once
#include "../kinematics/forward.h"

// ─── Central Pattern Generator ────────────────────────────────────────────────
// Each joint oscillates independently:
//   θi(t) = Ai · sin(ω·t + φi) + θi_offset
//
// Phase relationships for forward gait:
//   - Hip pitch L vs R:     φ = π  (anti-phase)
//   - Knee vs hip pitch:    φ = -π/4
//   - Hip roll vs pitch:    φ = π/2

struct CPGParams {
    float period_s;           // gait cycle period [s]

    // Per-joint amplitudes [rad]
    float hip_roll_amp;
    float hip_pitch_amp;
    float knee_amp;
    float ankle_roll_amp;

    // Per-joint phase offsets [rad] — applied on top of the L/R offset
    float hip_roll_phase_offset;
    float knee_phase_offset;

    // DC offsets (modified by stabilizer)
    float hip_pitch_offset_L;
    float hip_pitch_offset_R;
    float hip_roll_offset_L;    // cancels servo halt bias so oscillation centers at physical 0°
    float hip_roll_offset_R;
    float ankle_roll_offset_L;
    float ankle_roll_offset_R;
    float knee_offset_L;
    float knee_offset_R;
};

struct LegAngles {
    JointAngles left;
    JointAngles right;
};

class CPGOscillator {
public:
    CPGOscillator();
    void init(const CPGParams& params);

    // Update oscillator at current time [s], returns angles for both legs.
    LegAngles update(float t_s);

    // Apply offset corrections from stabilizer (called before update)
    void set_offsets(float pitch_offset_L, float pitch_offset_R,
                     float roll_offset_L,  float roll_offset_R);

    // Runtime parameter tuning (via WebSocket)
    void set_period(float period_s);
    void set_amplitude(float hip_roll, float hip_pitch, float knee, float ankle_roll);

    const CPGParams& params() const { return params_; }

private:
    CPGParams params_;
    float omega_;   // 2π/T
};
