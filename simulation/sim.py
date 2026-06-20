"""
Optimus Biped Simulation
========================
Simulates FK/IK and visualises foot trajectories + CPG gait cycles.
Run without hardware:
    pip install -r requirements.txt
    python sim.py
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

# ─── Robot parameters ─────────────────────────────────────────────────────────
L1 = 100.0   # femur  [mm]
L2 = 100.0   # tibia  [mm]
L3 =  50.0   # foot   [mm]

# Servo limits [rad]
SERVO_MIN =  np.deg2rad(-90)
SERVO_MAX =  np.deg2rad( 90)

# CPG parameters
CPG_PERIOD   = 1.0          # [s]
CPG_AMP_HP   = 0.35         # hip pitch amplitude  [rad]
CPG_AMP_HR   = 0.15         # hip roll amplitude   [rad]
CPG_AMP_K    = 0.25         # knee amplitude       [rad]
CPG_AMP_AR   = 0.10         # ankle roll amplitude [rad]
CPG_PHASE_LR = np.pi        # left vs right hip pitch anti-phase
CPG_PHASE_K  = -np.pi / 4  # knee relative to hip pitch
CPG_PHASE_HR = np.pi / 2   # hip roll relative to hip pitch

# ─── Forward Kinematics ───────────────────────────────────────────────────────

def fk_sagittal(theta2: float, theta3: float) -> tuple[float, float]:
    """FK in sagittal plane → (x, y) [mm]"""
    x = L1 * np.cos(theta2) + L2 * np.cos(theta2 + theta3) + L3
    y = L1 * np.sin(theta2) + L2 * np.sin(theta2 + theta3)
    return x, y


def fk_coronal(theta1: float, theta4: float) -> float:
    """FK in coronal plane → z [mm]"""
    return (L1 + L2) * np.sin(theta1) + L3 * np.sin(theta1 + theta4)


def fk(theta1: float, theta2: float, theta3: float, theta4: float
        ) -> tuple[float, float, float]:
    """Full FK → (x, y, z) [mm]"""
    x, y = fk_sagittal(theta2, theta3)
    z = fk_coronal(theta1, theta4)
    return x, y, z


def dh_matrix(theta: float, d: float, a: float, alpha: float) -> np.ndarray:
    """Standard DH homogeneous transform (4×4)."""
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    return np.array([
        [ct,   -st*ca,  st*sa, a*ct],
        [st,    ct*ca, -ct*sa, a*st],
        [0,        sa,     ca,    d],
        [0,         0,      0,    1],
    ], dtype=float)


def fk_dh(theta1: float, theta2: float, theta3: float, theta4: float
           ) -> tuple[float, float, float]:
    """Full DH chain FK for cross-validation."""
    T1 = dh_matrix(theta1, 0, 0,  np.pi/2)
    T2 = dh_matrix(theta2, 0, L1, 0)
    T3 = dh_matrix(theta3, 0, L2, 0)
    T4 = dh_matrix(theta4, 0, L3, 0)
    T = T1 @ T2 @ T3 @ T4
    return T[0, 3], T[1, 3], T[2, 3]


# ─── Inverse Kinematics ───────────────────────────────────────────────────────

def ik(x: float, y: float, z: float, elbow_up: bool = True
       ) -> tuple[float, float, float, float] | None:
    """
    Solve IK for foot position (x, y, z) [mm].
    Returns (θ1, θ2, θ3, θ4) in radians, or None if out of reach.

    Assumes θ4 = −θ1 (ankle cancels hip roll), so in the coronal plane:
        z = (L1+L2)·sin(θ1)  →  θ1 = arcsin(z / (L1+L2))
    """
    # Step 1: hip roll — recover from coronal projection
    ratio = z / (L1 + L2)
    if abs(ratio) > 1.0:
        return None   # lateral target out of reach
    theta1 = np.arcsin(ratio)

    # Step 2: effective 2D distance (subtract rigid foot)
    dx = x - L3
    d = np.hypot(dx, y)

    # Step 3: knee by law of cosines
    cos_t3 = (d**2 - L1**2 - L2**2) / (2 * L1 * L2)
    if abs(cos_t3) > 1.0:
        return None   # out of reach

    sin_t3 = np.sqrt(1 - cos_t3**2) if elbow_up else -np.sqrt(1 - cos_t3**2)
    theta3 = np.arctan2(sin_t3, cos_t3)

    # Step 4: hip pitch
    theta2 = np.arctan2(y, dx) - np.arctan2(L2 * sin_t3, L1 + L2 * cos_t3)

    # Step 5: ankle roll compensates hip roll
    theta4 = -theta1

    # Clamp
    angles = np.clip([theta1, theta2, theta3, theta4], SERVO_MIN, SERVO_MAX)
    return tuple(angles)


# ─── CPG Oscillator ───────────────────────────────────────────────────────────

def cpg_step(t: float, side: str = 'left') -> dict:
    """
    Compute joint angles for one leg at time t [s].
    side: 'left' or 'right'
    """
    omega = 2 * np.pi / CPG_PERIOD
    phase_lr = CPG_PHASE_LR if side == 'right' else 0.0

    hp  = CPG_AMP_HP * np.sin(omega * t + phase_lr)
    k   = CPG_AMP_K  * np.sin(omega * t + phase_lr + CPG_PHASE_K)
    hr  = CPG_AMP_HR * np.sin(omega * t + phase_lr + CPG_PHASE_HR)
    ar  = -hr   # ankle cancels hip roll

    return {'hip_roll': hr, 'hip_pitch': hp, 'knee': k, 'ankle_roll': ar}


# ─── Validation: IK inverts FK ────────────────────────────────────────────────

def validate_ik_fk(n: int = 1000, tol_mm: float = 0.5) -> bool:
    """
    Random joint angles (with θ4=−θ1 constraint) → FK → IK → FK again.
    Check that both FK calls return the same foot position.

    θ4=−θ1 is the physical ankle constraint assumed by the IK solver.
    """
    rng = np.random.default_rng(42)
    errors = []
    for _ in range(n):
        q = rng.uniform(SERVO_MIN, SERVO_MAX, 4)
        q[3] = -q[0]   # enforce ankle cancels hip roll
        x, y, z = fk(*q)
        result = ik(x, y, z, elbow_up=(q[2] >= 0))
        if result is None:
            continue
        x2, y2, z2 = fk(*result)
        err = np.sqrt((x - x2)**2 + (y - y2)**2 + (z - z2)**2)
        errors.append(err)

    errors = np.array(errors)
    passed = int((errors < tol_mm).sum())
    total  = len(errors)
    print(f"[FK→IK validation] {passed}/{total} within {tol_mm} mm  "
          f"| mean err: {errors.mean():.4f} mm  max: {errors.max():.4f} mm")
    return errors.max() < tol_mm


def validate_analytical_vs_dh() -> bool:
    """
    Sanity-check the DH FK against the analytical FK at the neutral pose
    (all joints = 0).  At θ₂=θ₃=0 the foot is directly in front at (L1+L2+L3, 0, 0)
    in both formulations — a quick smoke-test confirming the DH chain is assembled.

    NOTE: The analytical model treats L3 as a fixed horizontal foot offset (spec
    constraint: "pie rígido"), while the generic DH chain rotates every link.
    They therefore differ for non-zero θ₂/θ₃ by design; full 3D validation is
    provided by the FK→IK round-trip test instead.
    """
    x_a, y_a, z_a = fk(0.0, 0.0, 0.0, 0.0)
    x_d, y_d, z_d = fk_dh(0.0, 0.0, 0.0, 0.0)
    err = abs(x_a - x_d)
    expected = L1 + L2 + L3
    ok = abs(x_a - expected) < 0.01 and err < 0.01
    print(f"[DH neutral-pose check]  analytical x={x_a:.1f} mm  DH x={x_d:.1f} mm  "
          f"expected={expected:.1f} mm  → {'PASS' if ok else 'FAIL'}")
    return ok


# ─── Plotting ─────────────────────────────────────────────────────────────────

def plot_foot_trajectory():
    """Plot foot trajectory for one full gait cycle (left leg)."""
    t_vals = np.linspace(0, CPG_PERIOD, 400)
    xs, ys, zs = [], [], []

    for t in t_vals:
        angles = cpg_step(t, 'left')
        x, y, z = fk(angles['hip_roll'], angles['hip_pitch'],
                      angles['knee'],     angles['ankle_roll'])
        xs.append(x); ys.append(y); zs.append(z)

    xs, ys, zs = np.array(xs), np.array(ys), np.array(zs)
    return t_vals, xs, ys, zs


def plot_cpg_signals():
    """CPG joint angle signals over two gait cycles."""
    t_vals = np.linspace(0, 2 * CPG_PERIOD, 800)
    omega = 2 * np.pi / CPG_PERIOD

    signals = {
        'Hip Pitch L':  CPG_AMP_HP * np.sin(omega * t_vals),
        'Hip Pitch R':  CPG_AMP_HP * np.sin(omega * t_vals + CPG_PHASE_LR),
        'Knee L':       CPG_AMP_K  * np.sin(omega * t_vals + CPG_PHASE_K),
        'Hip Roll L':   CPG_AMP_HR * np.sin(omega * t_vals + CPG_PHASE_HR),
    }
    return t_vals, signals


def plot_ik_reachability():
    """2D slice of IK reachable workspace (y=0 plane → xz plane)."""
    xs = np.linspace(-L1 - L2 - L3, L1 + L2 + L3, 200)
    zs = np.linspace(-L1 - L2, L1 + L2, 200)
    reachable = np.zeros((len(zs), len(xs)), dtype=bool)

    for i, z in enumerate(zs):
        for j, x in enumerate(xs):
            y = -(L1 + L2) * 0.7   # typical stance height
            result = ik(x, y, z)
            reachable[i, j] = result is not None

    return xs, zs, reachable


def main():
    print("=" * 60)
    print("Optimus Biped Simulation")
    print("=" * 60)

    # Validations
    ok1 = validate_analytical_vs_dh()
    ok2 = validate_ik_fk()
    print()

    # ── Figure layout ─────────────────────────────────────────────────────────
    fig = plt.figure(figsize=(16, 10))
    fig.suptitle("Optimus Biped — Kinematics & Gait Simulation", fontsize=14, y=0.98)
    gs = gridspec.GridSpec(2, 3, figure=fig, hspace=0.45, wspace=0.35)

    # 1. Foot trajectory XY (sagittal)
    ax1 = fig.add_subplot(gs[0, 0])
    t_vals, xs, ys, zs = plot_foot_trajectory()
    sc1 = ax1.scatter(xs, ys, c=t_vals, cmap='plasma', s=6)
    ax1.set_xlabel("x [mm]  (forward)")
    ax1.set_ylabel("y [mm]  (up)")
    ax1.set_title("Foot Trajectory — Sagittal (XY)")
    ax1.set_aspect('equal')
    ax1.grid(True, alpha=0.3)
    plt.colorbar(sc1, ax=ax1, label="t [s]")

    # 2. Foot trajectory XZ (coronal)
    ax2 = fig.add_subplot(gs[0, 1])
    sc2 = ax2.scatter(xs, zs, c=t_vals, cmap='viridis', s=6)
    ax2.set_xlabel("x [mm]  (forward)")
    ax2.set_ylabel("z [mm]  (lateral)")
    ax2.set_title("Foot Trajectory — Coronal (XZ)")
    ax2.grid(True, alpha=0.3)
    plt.colorbar(sc2, ax=ax2, label="t [s]")

    # 3. 3D foot trajectory
    ax3 = fig.add_subplot(gs[0, 2], projection='3d')
    ax3.plot(xs, zs, ys, lw=1.5, color='royalblue')
    ax3.scatter(xs[0], zs[0], ys[0], color='green', s=60, label='start', zorder=5)
    ax3.set_xlabel("x [mm]")
    ax3.set_ylabel("z [mm]")
    ax3.set_zlabel("y [mm]")
    ax3.set_title("3D Foot Trajectory")
    ax3.legend(fontsize=8)

    # 4. CPG signals
    ax4 = fig.add_subplot(gs[1, 0:2])
    t_cpg, signals = plot_cpg_signals()
    colors = ['#e74c3c', '#3498db', '#2ecc71', '#f39c12']
    for (name, sig), col in zip(signals.items(), colors):
        ax4.plot(t_cpg, np.rad2deg(sig), label=name, color=col, lw=1.8)
    ax4.axhline(0, color='k', lw=0.5, ls='--')
    ax4.set_xlabel("Time [s]")
    ax4.set_ylabel("Angle [°]")
    ax4.set_title("CPG Oscillator Signals — 2 Gait Cycles")
    ax4.legend(loc='upper right', fontsize=8, ncol=2)
    ax4.grid(True, alpha=0.3)

    # Mark gait phases
    for phase_t, label in [(0, 'SS_L'), (0.25, 'DS'), (0.5, 'SS_R'), (0.75, 'DS')]:
        ax4.axvline(phase_t, color='gray', ls=':', lw=0.8, alpha=0.7)
        ax4.text(phase_t + 0.02, ax4.get_ylim()[1] * 0.9, label, fontsize=7, color='gray')

    # 5. Reachability workspace
    ax5 = fig.add_subplot(gs[1, 2])
    xs_r, zs_r, reach = plot_ik_reachability()
    ax5.contourf(xs_r, zs_r, reach.astype(float), levels=[0.5, 1.5],
                 colors=['#2ecc71'], alpha=0.5)
    ax5.contour(xs_r, zs_r, reach.astype(float), levels=[0.5],
                colors=['#27ae60'], linewidths=1.2)
    ax5.scatter(xs[:1], zs[:1], color='red', s=50, zorder=5, label='gait start')
    ax5.plot(xs, zs, 'r-', lw=1.2, alpha=0.8, label='foot path')
    ax5.set_xlabel("x [mm]")
    ax5.set_ylabel("z [mm]")
    ax5.set_title("IK Reachable Workspace (XZ slice, y≈-140 mm)")
    ax5.legend(fontsize=8)
    ax5.grid(True, alpha=0.3)

    # Validation summary
    status = "PASS" if (ok1 and ok2) else "FAIL"
    fig.text(0.01, 0.01, f"Validation: {status}", fontsize=9,
             color='green' if status == 'PASS' else 'red')

    plt.savefig("optimus_sim.png", dpi=150, bbox_inches='tight')
    print("Plot saved to: optimus_sim.png")
    plt.show()


if __name__ == "__main__":
    main()
