#include "cpg.h"
#include "../../include/config.h"
#include <math.h>

static constexpr float TWO_PI  = 6.28318530f;
static constexpr float DEG2RAD = 3.14159265f / 180.0f;

// Phase constants converted from degrees at compile time
static constexpr float BASE_PHASE = CPG_HIP_PITCH_BASE_PHASE_DEG * DEG2RAD;
static constexpr float R_PHASE    = CPG_PHASE_HIP_PITCH_R_DEG    * DEG2RAD;

CPGOscillator::CPGOscillator() {
    omega_ = TWO_PI / CPG_PERIOD_S;
}

void CPGOscillator::init(const CPGParams& p) {
    params_ = p;
    omega_  = TWO_PI / p.period_s;
}

void CPGOscillator::set_period(float period_s) {
    params_.period_s = period_s;
    omega_ = TWO_PI / period_s;
}

void CPGOscillator::set_amplitude(float hip_roll, float hip_pitch,
                                   float knee, float ankle_roll) {
    params_.hip_roll_amp   = hip_roll;
    params_.hip_pitch_amp  = hip_pitch;
    params_.knee_amp       = knee;
    params_.ankle_roll_amp = ankle_roll;
}

void CPGOscillator::set_offsets(float pitch_offset_L, float pitch_offset_R,
                                  float roll_offset_L,  float roll_offset_R) {
    params_.hip_pitch_offset_L  = pitch_offset_L;
    params_.hip_pitch_offset_R  = pitch_offset_R;
    params_.ankle_roll_offset_L = roll_offset_L;
    params_.ankle_roll_offset_R = roll_offset_R;
}

LegAngles CPGOscillator::update(float t_s) {
    float wt = omega_ * t_s;

    // ── Left leg ──────────────────────────────────────────────────────────────
    // hip pitch: base phase = BASE_PHASE (π = forward, 0 = backward)
    float hp_L  = params_.hip_pitch_amp
                * sinf(wt + BASE_PHASE)
                + params_.hip_pitch_offset_L;

    // knee: same base phase as hip pitch + knee offset relative to it
    float knee_L = params_.knee_amp
                 * sinf(wt + BASE_PHASE + params_.knee_phase_offset)
                 + params_.knee_offset_L;

    // hip roll: offset by π/2 relative to hip pitch + DC trim to center at physical 0°
    float hr_L  = params_.hip_roll_amp
                * sinf(wt + params_.hip_roll_phase_offset)
                + params_.hip_roll_offset_L;

    // ankle roll: compensate hip roll  (θ4 = -θ1)
    float ar_L  = -hr_L + params_.ankle_roll_offset_L;

    // ── Right leg ─────────────────────────────────────────────────────────────
    // hip pitch: base phase + anti-phase offset
    float hp_R  = params_.hip_pitch_amp
                * sinf(wt + BASE_PHASE + R_PHASE)
                + params_.hip_pitch_offset_R;

    float knee_R = params_.knee_amp
                 * sinf(wt + BASE_PHASE + R_PHASE + params_.knee_phase_offset)
                 + params_.knee_offset_R;

    float hr_R  = params_.hip_roll_amp
                * sinf(wt + R_PHASE + params_.hip_roll_phase_offset)
                + params_.hip_roll_offset_R;

    float ar_R  = -hr_R + params_.ankle_roll_offset_R;

    LegAngles out;
    out.left  = { hr_L, hp_L, knee_L, ar_L };
    out.right = { hr_R, hp_R, knee_R, ar_R };
    return out;
}
