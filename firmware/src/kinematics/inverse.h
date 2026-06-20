#pragma once
#include "forward.h"

enum class IKElbow { UP, DOWN };

// Result of IK solve — includes validity flag
struct IKResult {
    JointAngles angles;
    bool valid;   // false if target is out of reach
};

// Solve IK given desired foot position [x, y, z] in mm.
// Elbow mode: UP = knee forward, DOWN = knee backward.
//
// Steps:
//   1. θ1 = atan2(z, y_lateral)           — decouple hip roll
//   2. d  = sqrt((x-L3)^2 + y^2)          — effective 2D reach
//   3. cos(θ3) = (d^2 - L1^2 - L2^2) / (2·L1·L2)  — law of cosines
//   4. θ2 = atan2(y, x-L3) - atan2(L2·sin(θ3), L1+L2·cos(θ3))
//   5. θ4 = -θ1                            — ankle cancels hip roll
IKResult ik_solve(Vec3 target, float L1, float L2, float L3,
                  IKElbow elbow = IKElbow::UP);

// Clamp a joint angle to physical servo limits [deg]
float clamp_angle_rad(float angle_rad, float min_deg, float max_deg);
