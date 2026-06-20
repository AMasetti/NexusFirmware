#pragma once
#include "stabilizer.h"   // IMUEstimate; also pulls in cpg.h → JointAngles

// ─── IMU-gated weight-transfer walk ──────────────────────────────────────────
// 4-phase state machine:
//   SHIFT_RIGHT  lerp CoM to right foot (IMU-gated)
//   SWING_LEFT   float left leg forward  (timed, sinusoidal)
//   SHIFT_LEFT   lerp CoM to left foot  (IMU-gated)
//   SWING_RIGHT  float right leg forward (timed, sinusoidal)
//
// Leg strategy per swing:
//   float  hip_pitch = +WALK_SWING_HP_RAD  × quarter-sin   (lift + stride)
//   float  knee      = −WALK_SWING_KNEE_RAD × quarter-sin  (opposite dir = extra forward)
//   stance knee      = +WALK_STANCE_LOWER_RAD × half-sin   (lower body → forward propulsion)
//   stance ankle     = +SHIFT_ROLL_RAD  (compensates lateral hip tilt)
//   float  ankle     = −SHIFT_ROLL_RAD  (keeps float foot parallel to ground)
//
// Forward locomotion: at end of each swing the float foot has landed at hip_pitch=PEAK.
// The following SHIFT phase lerps that hip_pitch back to 0 while CoM moves over the foot —
// the servo tries to pull the hip backward, but with foot planted the body moves forward.

class WalkController {
public:
    struct Output {
        JointAngles left, right;
        float r_shoulder_fb_deg;
        float l_shoulder_fb_deg;
    };

    WalkController();
    void   init();
    Output update(const IMUEstimate& imu, float dt_s);

private:
    enum class Phase : uint8_t {
        SHIFT_RIGHT = 0,
        SWING_LEFT  = 1,
        SHIFT_LEFT  = 2,
        SWING_RIGHT = 3,
    };

    Phase   phase_;
    float   phase_t_;             // 0..1 normalised progress within phase
    Output  pose_start_;          // captured at each phase transition; lerp origin for SHIFT phases
    uint8_t stable_count_;        // consecutive cycles SHIFT imu_gate has been satisfied
    bool    swing_done_;          // swing timer elapsed; holding final pose for post-swing stability
    uint8_t post_stable_count_;   // consecutive stable cycles at end of swing

    // Which leg is floating right now (NONE during SHIFT phases)
public:
    enum class FloatLeg : uint8_t { NONE, LEFT, RIGHT };
    FloatLeg float_leg()        const;
    float    target_roll_rad()  const;  // walk's commanded lateral lean for this phase
private:
    bool   imu_gate(const IMUEstimate& imu) const;
    bool   imu_stable_post_swing(const IMUEstimate& imu) const;
    Output shift_target() const;
    Output swing_output(float t)  const;

    static Output lerp_output(const Output& a, const Output& b, float t);
    static float  smoothstep(float t);
};
