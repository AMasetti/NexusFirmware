#include "walk.h"
#include "../../include/config.h"
#include <math.h>
#include <string.h>

static constexpr float PI     = 3.14159265f;
static constexpr float DEG2RAD = PI / 180.0f;

// ─── helpers ─────────────────────────────────────────────────────────────────

float WalkController::smoothstep(float t) {
    if (t <= 0.0f) return 0.0f;
    if (t >= 1.0f) return 1.0f;
    return t * t * (3.0f - 2.0f * t);
}

WalkController::Output WalkController::lerp_output(const Output& a,
                                                     const Output& b,
                                                     float t) {
    auto lf = [t](float x, float y) { return x + t * (y - x); };
    Output o;
    o.left.hip_roll    = lf(a.left.hip_roll,    b.left.hip_roll);
    o.left.hip_pitch   = lf(a.left.hip_pitch,   b.left.hip_pitch);
    o.left.knee        = lf(a.left.knee,         b.left.knee);
    o.left.ankle_roll  = lf(a.left.ankle_roll,   b.left.ankle_roll);
    o.right.hip_roll   = lf(a.right.hip_roll,    b.right.hip_roll);
    o.right.hip_pitch  = lf(a.right.hip_pitch,   b.right.hip_pitch);
    o.right.knee       = lf(a.right.knee,         b.right.knee);
    o.right.ankle_roll = lf(a.right.ankle_roll,   b.right.ankle_roll);
    o.r_shoulder_fb_deg = lf(a.r_shoulder_fb_deg, b.r_shoulder_fb_deg);
    o.l_shoulder_fb_deg = lf(a.l_shoulder_fb_deg, b.l_shoulder_fb_deg);
    return o;
}

// ─── lifecycle ───────────────────────────────────────────────────────────────

WalkController::WalkController()
    : phase_(Phase::SHIFT_RIGHT), phase_t_(0.0f), stable_count_(0),
      swing_done_(false), post_stable_count_(0) {
    memset(&pose_start_, 0, sizeof(pose_start_));
}

void WalkController::init() {
    phase_             = Phase::SHIFT_RIGHT;
    phase_t_           = 0.0f;
    stable_count_      = 0;
    swing_done_        = false;
    post_stable_count_ = 0;
    memset(&pose_start_, 0, sizeof(pose_start_));
}

// ─── phase logic ─────────────────────────────────────────────────────────────

// Commanded lateral lean for this phase — used by task_walk to compute roll error
// so balance corrections assist weight transfer instead of fighting it.
float WalkController::target_roll_rad() const {
    const float sr = WALK_SHIFT_ROLL_DEG * DEG2RAD;
    switch (phase_) {
        case Phase::SHIFT_RIGHT:
        case Phase::SWING_LEFT:   return  sr;
        case Phase::SHIFT_LEFT:
        case Phase::SWING_RIGHT:  return -sr;
        default:                  return  0.0f;
    }
}

WalkController::FloatLeg WalkController::float_leg() const {
    if (phase_ == Phase::SWING_LEFT)  return FloatLeg::LEFT;
    if (phase_ == Phase::SWING_RIGHT) return FloatLeg::RIGHT;
    return FloatLeg::NONE;
}

bool WalkController::imu_gate(const IMUEstimate& imu) const {
    const float thr = WALK_SHIFT_THRESHOLD_DEG * DEG2RAD;
    switch (phase_) {
        case Phase::SHIFT_RIGHT: return imu.roll_rad >  thr;
        case Phase::SHIFT_LEFT:  return imu.roll_rad < -thr;
        default:                 return false;
    }
}

// Body is settled when pitch is near zero and roll is close to the commanded lean.
bool WalkController::imu_stable_post_swing(const IMUEstimate& imu) const {
    const float thr      = WALK_POST_SWING_STABLE_DEG * DEG2RAD;
    const float roll_err = imu.roll_rad - target_roll_rad();
    return (fabsf(imu.pitch_rad) < thr) && (fabsf(roll_err) < thr);
}

// Target pose for SHIFT phases (where we want the CoM to land).
// hip_pitch = 0 for both: the stance foot roll-over is handled by lerping
// from pose_start_ (which captures the foot's landing hp) back to 0 here.
WalkController::Output WalkController::shift_target() const {
    Output t;
    memset(&t, 0, sizeof(t));
    const float sr = WALK_SHIFT_ROLL_DEG * DEG2RAD;
    if (phase_ == Phase::SHIFT_RIGHT) {
        t.left.hip_roll    = -sr;   // pelvis tilts right → right foot loads
        t.right.hip_roll   =  sr;
        t.left.ankle_roll  =  sr;   // keep soles flat
        t.right.ankle_roll = -sr;
    } else {
        t.left.hip_roll    =  sr;
        t.right.hip_roll   = -sr;
        t.left.ankle_roll  = -sr;
        t.right.ankle_roll =  sr;
    }
    return t;
}

// Analytical swing output — both swing phases use a sinusoidal profile.
// Float leg:
//   hip_pitch = +HP × quarter-sin(t)   → 0 at start, HP at landing
//   knee      = −KA × quarter-sin(t)   → opposite direction = extra forward
// Stance leg:
//   knee      = +KS × half-sin(t)      → peaks mid-swing then returns 0
//   ankle     compensates hip roll
// Arms swing opposite to float leg (human gait).
WalkController::Output WalkController::swing_output(float t) const {
    Output o;
    memset(&o, 0, sizeof(o));

    const float sr  = WALK_SHIFT_ROLL_DEG   * DEG2RAD;
    const float hp  = WALK_SWING_HP_DEG    * DEG2RAD;
    const float ka  = WALK_SWING_KNEE_DEG  * DEG2RAD;
    const float ks  = WALK_STANCE_LOWER_DEG * DEG2RAD;
    const float arm = WALK_ARM_SWING_DEG;

    // quarter-sin (0→1): smooth monotonic rise, foot lands at peak
    float lift = sinf(PI * t / 2.0f);
    // Stance knee is delayed until the float leg's hip pitch has largely completed.
    // kneel_t remaps [DELAY..1] → [0..1] so the knee does a full half-sine within
    // the remaining window and returns to 0 when the foot plants.
    const float delay   = WALK_SWING_KNEE_DELAY;
    float       kneel_t = (t > delay) ? (t - delay) / (1.0f - delay) : 0.0f;
    float       peak    = sinf(PI * kneel_t);

    if (phase_ == Phase::SWING_LEFT) {
        // weight maintained on right foot
        o.left.hip_roll    = -sr;
        o.right.hip_roll   =  sr;
        o.left.ankle_roll  =  sr;   // float foot stays flat (compensates hip_roll)
        o.right.ankle_roll = -sr;   // stance ankle compensates

        // float left leg
        o.left.hip_pitch   =  hp * lift;
        o.left.knee        =  ka * lift;   // flex knee → foot lifts off ground

        // stance right leg lowers mid-swing for propulsion
        o.right.knee       =  ks * peak;

        // SWING_LEFT (left leg forward) → right arm leads (opposite side)
        o.r_shoulder_fb_deg = -arm * peak;
        o.l_shoulder_fb_deg =  arm * peak;

    } else {  // SWING_RIGHT
        // weight maintained on left foot
        o.left.hip_roll    =  sr;
        o.right.hip_roll   = -sr;
        o.left.ankle_roll  = -sr;   // stance ankle compensates
        o.right.ankle_roll =  sr;   // float foot stays flat (compensates hip_roll)

        // float right leg
        o.right.hip_pitch  =  hp * lift;
        o.right.knee       =  ka * lift;   // flex knee → foot lifts off ground

        // stance left leg lowers mid-swing
        o.left.knee        =  ks * peak;

        // SWING_RIGHT (right leg forward) → left arm leads (opposite side)
        o.r_shoulder_fb_deg =  arm * peak;
        o.l_shoulder_fb_deg = -arm * peak;
    }

    return o;
}

// ─── main update ─────────────────────────────────────────────────────────────

WalkController::Output WalkController::update(const IMUEstimate& imu, float dt_s) {
    float tc = fminf(phase_t_, 1.0f);
    float s  = smoothstep(tc);

    bool is_shift = (phase_ == Phase::SHIFT_RIGHT || phase_ == Phase::SHIFT_LEFT);

    Output out;
    if (is_shift) {
        out = lerp_output(pose_start_, shift_target(), s);
    } else {
        out = swing_output(tc);
    }

    // ── Post-swing stability hold ─────────────────────────────────────────────
    // Swing timer has fired; hold the landed pose until pitch and roll settle.
    // task_walk's balance overlay continues running every cycle, actively correcting.
    if (!is_shift && swing_done_) {
        bool stable_now = imu_stable_post_swing(imu);
        if (stable_now) { if (post_stable_count_ < 255) ++post_stable_count_; }
        else            { post_stable_count_ = 0; }

        if (post_stable_count_ >= WALK_POST_SWING_STABLE_CYCLES) {
            post_stable_count_ = 0;
            swing_done_        = false;
            stable_count_      = 0;
            pose_start_        = out;
            phase_             = static_cast<Phase>((static_cast<uint8_t>(phase_) + 1) % 4);
            phase_t_           = 0.0f;
        }
        return out;
    }

    // ── Advance phase_t_ ─────────────────────────────────────────────────────
    float dur = is_shift ? WALK_SHIFT_DURATION_S : WALK_SWING_DURATION_S;
    phase_t_ += dt_s / dur;

    // ── Swing timer done → enter post-swing stability hold ───────────────────
    if (!is_shift && phase_t_ >= 1.0f) {
        phase_t_           = 1.0f;   // freeze at landed pose
        swing_done_        = true;
        post_stable_count_ = 0;
        return out;
    }

    // ── SHIFT phase: hold at target; no timeout — only IMU stability advances ─
    // task_walk's balance overlay adjusts servo setpoints every cycle while we wait.
    if (is_shift) {
        if (phase_t_ > 1.0f) phase_t_ = 1.0f;   // clamp: hold target lean indefinitely

        if (phase_t_ >= 0.40f) {   // minimum dwell before IMU gate is checked
            bool imu_ok = imu_gate(imu);
            if (imu_ok) { if (stable_count_ < 255) ++stable_count_; }
            else        { stable_count_ = 0; }

            if (stable_count_ >= WALK_SHIFT_STABLE_CYCLES) {
                stable_count_ = 0;
                pose_start_   = out;
                phase_        = static_cast<Phase>((static_cast<uint8_t>(phase_) + 1) % 4);
                phase_t_      = 0.0f;
            }
        }
    }

    return out;
}
