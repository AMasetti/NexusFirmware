#include "forward.h"
#include <string.h>

// ─── Mat4 ─────────────────────────────────────────────────────────────────────

Mat4 Mat4::identity() {
    Mat4 r;
    memset(&r, 0, sizeof(r));
    r.m[0][0] = r.m[1][1] = r.m[2][2] = r.m[3][3] = 1.0f;
    return r;
}

Mat4 Mat4::operator*(const Mat4& b) const {
    Mat4 c;
    memset(&c, 0, sizeof(c));
    for (int i = 0; i < 4; i++)
        for (int j = 0; j < 4; j++)
            for (int k = 0; k < 4; k++)
                c.m[i][j] += m[i][k] * b.m[k][j];
    return c;
}

// ─── DH transform ─────────────────────────────────────────────────────────────
// Standard DH convention:
//   Rz(theta) · Tz(d) · Tx(a) · Rx(alpha)
Mat4 dh_transform(float theta, float d, float a, float alpha) {
    float ct = cosf(theta), st = sinf(theta);
    float ca = cosf(alpha), sa = sinf(alpha);

    Mat4 T;
    T.m[0][0] = ct;       T.m[0][1] = -st*ca;   T.m[0][2] =  st*sa;   T.m[0][3] = a*ct;
    T.m[1][0] = st;       T.m[1][1] =  ct*ca;   T.m[1][2] = -ct*sa;   T.m[1][3] = a*st;
    T.m[2][0] = 0.0f;     T.m[2][1] =  sa;      T.m[2][2] =  ca;      T.m[2][3] = d;
    T.m[3][0] = 0.0f;     T.m[3][1] = 0.0f;     T.m[3][2] = 0.0f;     T.m[3][3] = 1.0f;
    return T;
}

// ─── Full DH chain ────────────────────────────────────────────────────────────
// DH parameters for 4-DOF leg (hip_roll, hip_pitch, knee, ankle_roll):
//
//  i | theta | d  | a   | alpha
//  1 | θ1    | 0  | 0   | π/2   (hip roll: rotates coronal plane)
//  2 | θ2    | 0  | L1  | 0     (hip pitch: sagittal, femur)
//  3 | θ3    | 0  | L2  | 0     (knee: sagittal, tibia)
//  4 | θ4    | 0  | L3  | 0     (ankle roll: foot)

Vec3 fk_full_dh(const JointAngles& q, float L1, float L2, float L3) {
    const float PI_2 = 1.57079633f;

    Mat4 T1 = dh_transform(q.hip_roll,   0.0f, 0.0f, PI_2);
    Mat4 T2 = dh_transform(q.hip_pitch,  0.0f, L1,   0.0f);
    Mat4 T3 = dh_transform(q.knee,       0.0f, L2,   0.0f);
    Mat4 T4 = dh_transform(q.ankle_roll, 0.0f, L3,   0.0f);

    Mat4 T = T1 * T2 * T3 * T4;

    Vec3 foot;
    foot.x = T.m[0][3];
    foot.y = T.m[1][3];
    foot.z = T.m[2][3];
    return foot;
}

// ─── Analytical FK (per-plane decomposition) ──────────────────────────────────
// Faster than full DH; used in real-time loop.
Vec3 fk_solve(const JointAngles& q, float L1, float L2, float L3) {
    // Sagittal plane: x (forward), y (up/down)
    float x = L1 * cosf(q.hip_pitch) +
               L2 * cosf(q.hip_pitch + q.knee) +
               L3;

    float y = L1 * sinf(q.hip_pitch) +
               L2 * sinf(q.hip_pitch + q.knee);

    // Coronal plane: z (lateral)
    // With ankle compensation θ4 = -θ1 → foot stays horizontal
    float z = (L1 + L2) * sinf(q.hip_roll) +
               L3 * sinf(q.hip_roll + q.ankle_roll);

    return {x, y, z};
}
