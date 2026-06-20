#pragma once
#include <math.h>

// ─── Forward Kinematics ───────────────────────────────────────────────────────
// Computes foot position in 3D space given joint angles.
//
// Sagittal plane (θ2, θ3):
//   x = L1·cos(θ2) + L2·cos(θ2+θ3) + L3
//   y = L1·sin(θ2) + L2·sin(θ2+θ3)
//
// Coronal plane (θ1, θ4):
//   z = L_offset · sin(θ1) + L3 · sin(θ1 + θ4)
//
// Full homogeneous transform: T_foot = T1(θ1)·T2(θ2)·T3(θ3)·T4(θ4)

struct Vec3 {
    float x, y, z;
};

struct JointAngles {
    float hip_roll;    // θ1  [rad]
    float hip_pitch;   // θ2  [rad]
    float knee;        // θ3  [rad]
    float ankle_roll;  // θ4  [rad]
};

// DH matrix (4x4 row-major) — only needed internally, exposed for testing
struct Mat4 {
    float m[4][4];
    static Mat4 identity();
    Mat4 operator*(const Mat4& b) const;
};

// Compute foot position given joint angles and link lengths (mm).
Vec3 fk_solve(const JointAngles& q, float L1, float L2, float L3);

// Build individual DH transform for joint i
Mat4 dh_transform(float theta, float d, float a, float alpha);

// Extract foot position from full DH chain
Vec3 fk_full_dh(const JointAngles& q, float L1, float L2, float L3);
