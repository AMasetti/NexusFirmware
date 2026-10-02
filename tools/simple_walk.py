"""
simple_walk.py — weight-shifting CPG walk for Optimus. No RL, no policy.

Standalone: shares nothing with rl_runner.py or mujuco/optimus/rl/cpg.py, so
tuning this cannot break the walking policy. Same WebSocket protocol as
rl_runner.py, so it runs remotely today and ports to firmware later — the gait
is pure arithmetic on a phase variable, no numpy, no model.

THE IDEA
    Shift the body fully onto one leg, then step with the other while it hangs
    free. The old CPG never did this (it held both hip_roll at 0), so neither
    foot ever unloaded and the swing leg dragged. Wider feet make the shift
    safe: the support polygon is now wide enough to stand on one foot.

FOUR PHASES per half-cycle, each a fraction of the gait period:

    SHIFT_L    roll the body over the LEFT foot        (both feet down)
    SWING_R    left carries the weight, right steps    (right foot up)
    SHIFT_R    roll the body over the RIGHT foot       (both feet down)
    SWING_L    right carries the weight, left steps    (left foot up)

    |<-- SHIFT_L -->|<-- SWING_R -->|<-- SHIFT_R -->|<-- SWING_L -->|
    0              0.25            0.5             0.75            1.0

Everything is driven by `phase` in [0,1). Each phase interpolates with a smooth
cosine ramp so the servos never see a step input (they buzz and overshoot on
step commands).

Usage:
    python simple_walk.py --dry-run          # print the gait table, send nothing
    python simple_walk.py --steps 4          # walk 4 steps then stand
    python simple_walk.py --period 2.0 --shift 20 --lift 25
    python simple_walk.py --no-imu           # disable IMU trim (pure open loop)

Start with --dry-run, then a slow --period 2.5 while holding the robot.
"""

import argparse
import json
import math
import threading
import time

import websocket  # pip install websocket-client

# ── Gait parameters (degrees / seconds) ──────────────────────────────────────

PERIOD_S = 2.0      # one full cycle: two steps

SHIFT_DEG = 18.0    # hip_roll amplitude — how far the body leans onto a foot.
                    # Geometry: the hips are 84 mm apart and the CoM sits
                    # 209 mm up, so the body must tilt atan(42/209) = 11.3 deg
                    # to put the CoM over one foot. Measured command->roll
                    # ratio on hardware is ~0.6, so ~18 deg of command is what
                    # actually delivers that 11 deg of body roll. At 10 the CoM
                    # only moved ~26 mm of the 42 mm needed, which is why the
                    # swing foot never fully unloaded.
                    # This is the parameter that decides whether the swing foot
                    # actually unloads. Too small and it drags; too large and
                    # the robot tips over the outer edge of the stance foot.

LIFT_DEG = 30.0     # knee flexion of the swing leg — how high the foot rises.
                    # Capped so STANCE_KNEE + LIFT stays under the firmware's
                    # 70 deg knee clamp (40 + 30 = 70 exactly at peak).
STEP_DEG = 20.0     # hip_pitch swing — how far forward the swing leg reaches

# The loaded leg EXTENDS while the free leg FLEXES. That difference in leg
# length is what actually drives the body forward: without it the swing foot
# just travels through the air and lands where it started.
STANCE_KNEE_DEG = 40.0   # baseline crouch, both legs. Deeper than before:
                         # at 28 both legs stayed nearly straight, so there was
                         # no visible flexion and little room to push from.
STANCE_EXTEND_DEG = 36.0  # how much the weight-bearing leg straightens
                          # (STANCE_KNEE - this = the extended leg's knee, so
                          # the loaded leg reaches 4 deg = essentially straight).
                          # Raised from 28: a straighter stance leg lifts the
                          # body higher over that foot, which both unloads the
                          # swing foot and gives more push-off travel.

# Ankle push-off: the stance ankle plantarflexes as weight leaves it, breaking
# static friction so the foot releases instead of sticking to the floor.
TOE_OFF_DEG = 14.0

# Heel roll applied to the swing (airborne) foot to break floor friction so it
# does not drag forward. Mirrored L/R like the other ankle terms.
HEEL_LIFT_DEG = 16.0

# Fraction of the swing quarter spent waiting for the stance leg to finish
# extending before the other foot leaves the ground. 0 = lift immediately
# (the old behaviour), 0.5 = wait half the quarter.
EXTEND_LEAD = 0.55

# How far ahead of the weight shift the receiving leg starts extending, as a
# fraction of the full cycle. The knee must already be straight when the load
# arrives; extending against a rising load stalls these servos.
EXTEND_ADVANCE = 0.12

# How quickly the swing leg completes its forward reach, as a multiple of the
# airborne window. >1 means it arrives forward before touchdown and waits there
# with the foot already placed ahead of the body.
SWING_REACH_RATE = 1.8

ANKLE_FOLLOW = 0.85  # ankle_roll counter-rotation as a fraction of hip_roll.
                     # 1.0 keeps the sole perfectly flat; slightly under 1.0
                     # lets the foot edge into the lean, which helps the shift.

ARM_SWING_DEG = 14.0  # arms counter-swing the opposite leg (yaw damping)

# IMU trim: small corrections, deliberately weak. This gait is meant to work
# open-loop; the IMU only nudges it back when it drifts.
IMU_ROLL_GAIN = 0.25
IMU_PITCH_GAIN = 0.30
IMU_TRIM_LIMIT_DEG = 8.0

# Standing-still IMU bias, degrees. Measured 2026-08-15 with the robot held
# upright and motionless: roll +7.4, pitch +7.0, drifting less than 1.2 deg
# across 337 samples — a fixed mounting/accelerometer offset, not noise. The
# firmware's calibrate_imu command does not remove it (it zeroes the gyro, not
# the accelerometer's absolute orientation).
#
# This matters more than it looks: apply_imu_trim() was reading a permanent
# +7.4 deg of roll and steadily pushing the ankles to "correct" it, biasing the
# robot to one side for the whole run. Subtracting the bias removes that.
# Re-measure with --measure-bias if the IMU is ever remounted.
IMU_ROLL_BIAS = 7.3
IMU_PITCH_BIAS = 7.2

# How much actual body roll a degree of commanded hip lean produces. The hips
# rotate the legs, not the torso directly, so the body tilts by less than the
# commanded angle. Used to tell the IMU trim what roll to expect, so it does
# not mistake a deliberate lean for a fall. Rough estimate — lower it if the
# trim still resists the lean, raise it if the robot over-leans.
EXPECTED_ROLL_RATIO = 0.7

# Standing lean offset, degrees of commanded hip lean.
# Measured 2026-08-15 with the robot standing and lean commanded at 0: the body
# still sat ~6.5 deg to one side (roll -7.0, -5.8, -6.7 across three samples,
# IMU bias already removed). A sweep from +22 to -22 produced roll +4.6 down to
# -12.9 — a healthy ~17 deg of travel, but centred on -4 instead of 0.
# This constant re-centres the lean so both sides get equal authority; without
# it one direction starts half way there and the other never catches up.
LEAN_TRIM_DEG = 6.5

# Fraction of the sideways lean produced by the ANKLES rather than the hips.
# Ankle roll tips the body over a planted foot without changing the distance
# between the feet; hip roll tilts but also splays the stance, which pushes the
# CoM target away faster than the lean brings the CoM toward it.
# 0.0 = all hips (old behaviour, stance opens up), 1.0 = all ankles.
ANKLE_LEAN_SHARE = 0.65

# Firmware soft limits (optimus/firmware-rl/include/config.h), degrees.
LIMITS = {
    "hip_roll": 45.0, "hip_pitch": 60.0, "knee": 70.0, "ankle_roll": 90.0,
    "shoulder_fb": 120.0, "shoulder_lat": 180.0, "forearm_lat": 90.0,
    "hip_yaw": 45.0,
}

HW_JOINT_NAMES = [
    "l_hip_roll", "l_hip_pitch", "l_knee", "l_ankle_roll",
    "r_hip_roll", "r_hip_pitch", "r_knee", "r_ankle_roll",
    "l_shoulder_fb", "r_shoulder_fb", "l_shoulder_lat", "r_shoulder_lat",
    "l_forearm_lat", "r_forearm_lat", "hip_yaw",
]

# Arms hang at the sides (firmware neutral is a T-pose).
ARM_LAT = {"l_shoulder_lat": -90.0, "r_shoulder_lat": +90.0}
# shoulder_fb: firmware config.h has ARM_SERVO_DIR_*_SHOULDER_FB = +1 on BOTH
# sides, so it applies no mirroring. Hardware shows the shoulder servos are
# mounted symmetrically, meaning equal signs already swing the arms in opposite
# physical directions — sending opposite signs (as an earlier version did) made
# both arms move the same way instead of counter-swinging.
ARM_FB_SIGN = {"l": +1.0, "r": +1.0}

# hip_pitch: firmware has SERVO_DIR_L_HIP_PITCH = +1 and
# SERVO_DIR_R_HIP_PITCH = -1, i.e. the firmware ALREADY mirrors the two legs.
# Sending both legs the same sign convention therefore makes one leg reach
# forward while the other reaches backward — the swing leg was retreating
# instead of stepping. Pre-invert the right leg here to cancel that.
HIP_PITCH_SIGN = {"l": +1.0, "r": -1.0}

CTRL_HZ = 50.0
CTRL_DT = 1.0 / CTRL_HZ


def ramp(t):
    """Smooth 0->1 over t in [0,1]. Zero slope at both ends: no step inputs."""
    t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
    return 0.5 - 0.5 * math.cos(math.pi * t)


def bump(t):
    """Smooth 0->1->0 over t in [0,1]. Used for lift and reach."""
    t = 0.0 if t < 0.0 else (1.0 if t > 1.0 else t)
    return math.sin(math.pi * t)


def clamp(name, deg):
    for key, lim in LIMITS.items():
        if name.endswith(key):
            return max(-lim, min(lim, deg))
    return deg


def gait(phase):
    """
    Joint angles in DEGREES for a phase in [0,1).

    Returns a plain dict — no numpy, so this ports to C almost line for line.
    """
    # Which quarter of the cycle are we in, and how far through it?
    quarter = int(phase * 4) % 4
    local = (phase * 4) % 1.0

    # ── Weight shift: hip_roll leans the body over one foot ──────────────────
    # +shift leans onto the LEFT foot, -shift onto the RIGHT.
    # Symmetric lean. The first quarter previously ramped 0 -> +SHIFT while the
    # third crossed +SHIFT -> -SHIFT, so the cycle spent more time leaning right
    # than left (mean +1.3 deg) and the left leg never fully took the weight.
    # Both transitions now cross the full -SHIFT..+SHIFT span.
    if quarter == 0:      # SHIFT_L: right -> left
        lean = SHIFT_DEG * (-1.0 + 2.0 * ramp(local))
    elif quarter == 1:    # SWING_R: hold on left while right steps
        lean = SHIFT_DEG
    elif quarter == 2:    # SHIFT_R: left -> right
        lean = SHIFT_DEG * (1.0 - 2.0 * ramp(local))
    else:                 # SWING_L: hold on right while left steps
        lean = -SHIFT_DEG

    # ── Swing gate: the free foot only leaves the ground after the stance leg
    #    has had time to finish extending. Both the lift AND the forward reach
    #    are driven by this, so the leg does not scrape forward while still
    #    loaded — that was burning the whole stride on the ground and left the
    #    robot rocking side to side without stepping.
    if quarter in (1, 3):
        swing_gate = 0.0 if local < EXTEND_LEAD else \
            (local - EXTEND_LEAD) / (1.0 - EXTEND_LEAD)
    else:
        swing_gate = 0.0

    swing_r = bump(swing_gate) if quarter == 1 else 0.0
    swing_l = bump(swing_gate) if quarter == 3 else 0.0

    # ── Forward reach ────────────────────────────────────────────────────────
    # +1 = foot forward of the hip, -1 = foot behind it. The swing leg's sweep
    # is gated by swing_gate so it only travels while airborne.
    # The swing leg must arrive forward EARLY and then hold there, so the foot
    # is already out in front when it lands. Ramping it across the whole swing
    # meant it only reached full extension at the instant of touchdown, so the
    # step never actually placed the foot ahead of the body.
    # The stance leg continues from +1 (where the shift left it) down to -1
    # across its own swing quarter — a continuous push-off, not a jump. The
    # previous form restarted it at 0 and stepped -20 deg in one control tick.
    if quarter == 1:                       # right swings front, left pushes
        reach_r = -1.0 + 2.0 * ramp(min(1.0, swing_gate * SWING_REACH_RATE))
        reach_l = 1.0 - 2.0 * local
    elif quarter == 3:                     # left swings front, right pushes
        reach_l = -1.0 + 2.0 * ramp(min(1.0, swing_gate * SWING_REACH_RATE))
        reach_r = 1.0 - 2.0 * local
    # During the double-support shifts, only the leg that is GIVING UP the load
    # keeps travelling back (it is pushing off); the leg RECEIVING the load
    # holds its forward position so it can accept the body over it.
    #
    # Previously both legs swept backwards through the whole shift, so for half
    # of every cycle nothing pushed forward and the body fell rearward — the
    # measured pitch was negative in all four phases (mean -1.7 deg) and the
    # robot drifted backwards.
    elif quarter == 0:                     # shifting onto L: R pushes off
        reach_l = 1.0                      # L already forward, holds to receive
        reach_r = -1.0                     # R stays back, finishing its push
    else:                                  # quarter == 2: shifting onto R
        reach_r = 1.0
        reach_l = -1.0

    # ── Load: which leg is carrying the body right now, 0..1 ────────────────
    # 1.0 = fully loaded (extends), 0.0 = free (flexes and steps).
    # Loaded follows the lean: leaning left means the left leg carries.
    load_l = 0.5 + 0.5 * (lean / SHIFT_DEG) if SHIFT_DEG else 0.5
    load_r = 1.0 - load_l

    # Extend the receiving leg BEFORE the weight arrives, not while it arrives.
    # Driving extension from `load` means the knee has to straighten against a
    # rising load — the servo stalls, arrives late, and then snaps straight once
    # the load leaves. Leading the phase by EXTEND_ADVANCE makes the leg reach
    # its extended length first and simply receive the weight already stiff.
    lead_phase = (phase + EXTEND_ADVANCE) % 1.0
    lead_q = int(lead_phase * 4) % 4
    lead_local = (lead_phase * 4) % 1.0
    if lead_q == 0:
        lead_lean = SHIFT_DEG * (-1.0 + 2.0 * ramp(lead_local))
    elif lead_q == 1:
        lead_lean = SHIFT_DEG
    elif lead_q == 2:
        lead_lean = SHIFT_DEG * (1.0 - 2.0 * ramp(lead_local))
    else:
        lead_lean = -SHIFT_DEG
    ext_l = 0.5 + 0.5 * (lead_lean / SHIFT_DEG) if SHIFT_DEG else 0.5
    ext_r = 1.0 - ext_l

    j = {n: 0.0 for n in HW_JOINT_NAMES}

    # Hip roll: OPPOSITE signs on the two legs.
    #
    # docs/JOINTS.md claims both legs take the same sign ("both servos raw (+)
    # push the CoM in the same direction"), but that does not match this robot:
    # with equal signs the hips adduct and abduct together, closing the stance
    # until the feet collide and then splaying it open again — no sideways
    # translation at all. The doc describes the intended wiring, not what is
    # actually mounted. Opposite signs produce a real weight shift.
    # Opposite hip signs tilt the body, but they ALSO splay the stance: one hip
    # abducts while the other adducts. Measured against the geometry, a lean
    # command of 18 deg shifts the CoM ~40 mm while pushing the feet ~129 mm
    # further apart — the target moves away three times faster than the CoM
    # approaches it, so the robot ends up crouched between its feet and never
    # over either one. Raising SHIFT_DEG makes this worse, not better.
    #
    # So the hips only supply a small part of the tilt, and ANKLE_LEAN_SHARE of
    # it comes from the ankles instead, which roll the body over a foot without
    # changing the distance between the feet.
    lean_cmd = (lean + LEAN_TRIM_DEG) * (1.0 - ANKLE_LEAN_SHARE)
    j["l_hip_roll"] = lean_cmd
    j["r_hip_roll"] = -lean_cmd

    # Ankles roll WITH the lean, not against it. When the body tips onto a foot,
    # that ankle has to rotate the same way so the sole stays flat under the
    # tilted body — counter-rotating (the previous behaviour) lifted the very
    # edge that needed to carry the load, so the robot stood on the outside of
    # its foot and could never plant the weight properly.
    #
    # Ankle roll is the NEGATIVE of hip roll — docs/JOINTS.md: "in IK,
    # ankle_roll = -hip_roll. The opposing set_inverse flags on each leg's
    # ankle vs. hip are what make this single equation work correctly for both
    # legs." This is what keeps the sole flat while the leg tilts: driving the
    # ankle the same way as the hip (the previous code) rolled the foot onto
    # its edge, so the stance heel pointed away from the body and the weight
    # never settled over that foot.
    # Both ankles roll the SAME physical way to tip the whole body sideways
    # over the stance foot — this is the part of the lean that does not widen
    # the stance. The mirrored SERVO_DIR means opposite signs on the wire.
    ankle_lean = (lean + LEAN_TRIM_DEG) * ANKLE_LEAN_SHARE
    j["l_ankle_roll"] = -lean_cmd * ANKLE_FOLLOW + ankle_lean
    j["r_ankle_roll"] = +lean_cmd * ANKLE_FOLLOW - ankle_lean

    # ── Knees: the loaded leg EXTENDS, the swinging leg FLEXES ──────────────
    # Extending under load pushes the body up and over the stance foot; flexing
    # the free leg shortens it so it can clear the ground and reach forward.
    # This length difference is what converts the arm-waving into travel.
    j["l_knee"] = (STANCE_KNEE_DEG
                   - STANCE_EXTEND_DEG * ext_l
                   + LIFT_DEG * swing_l)
    j["r_knee"] = (STANCE_KNEE_DEG
                   - STANCE_EXTEND_DEG * ext_r
                   + LIFT_DEG * swing_r)

    # Hip pitch: reach forward with the swing leg, push back with the stance.
    # HIP_PITCH_SIGN cancels the firmware's own L/R mirroring so that a positive
    # reach means "forward" on both legs.
    j["l_hip_pitch"] = STEP_DEG * reach_l
    j["r_hip_pitch"] = STEP_DEG * reach_r

    # ── Swing-foot heel lift ────────────────────────────────────────────────
    # Roll the AIRBORNE foot's heel up so it breaks free of the floor instead
    # of scuffing along it. Gated by swing_gate so it only acts once the foot
    # is genuinely unloaded, and bump() eases it back to flat before touchdown
    # so the foot lands on its whole sole.
    heel = HEEL_LIFT_DEG * bump(swing_gate)
    if quarter == 1:      # right foot is airborne
        j["r_ankle_roll"] += heel
    elif quarter == 3:    # left foot is airborne — mirrored sign
        j["l_ankle_roll"] -= heel

    # ── Toe-off: the ankle of the leg about to unload plantarflexes, which
    #    breaks static friction so the foot releases cleanly instead of
    #    sticking and dragging the robot backwards.
    # Fires during the shift phases, on the leg losing its load. bump() (not
    # ramp()) so it returns to zero by the end of the quarter — ramping left it
    # at full +14 deg and the next quarter dropped it to zero in one tick,
    # a 14 deg/tick jolt right at the moment of weight transfer.
    if quarter == 0:      # shifting onto L -> right foot is releasing
        j["r_ankle_roll"] += TOE_OFF_DEG * bump(local)
    elif quarter == 2:    # shifting onto R -> left foot is releasing
        j["l_ankle_roll"] += TOE_OFF_DEG * bump(local)

    # Arms counter-swing the opposite leg to damp body yaw.
    arm = ARM_SWING_DEG * (swing_r - swing_l)
    j["l_shoulder_fb"] = arm * ARM_FB_SIGN["l"]
    j["r_shoulder_fb"] = arm * ARM_FB_SIGN["r"]
    j.update(ARM_LAT)

    return j


def apply_imu_trim(j, pitch_deg, roll_deg, expected_roll_deg=0.0):
    """
    Weak IMU correction on top of the open-loop gait.

    The roll term corrects the error against the lean the gait is CURRENTLY
    ASKING FOR, not against zero. Correcting toward zero fights the gait: when
    the robot deliberately leans left, a zero-seeking trim reads that lean as a
    fall and pushes it back right — which is exactly what "it starts to lean
    left and then comes back right" looks like on hardware.
    """
    lim = IMU_TRIM_LIMIT_DEG
    roll_err = roll_deg - expected_roll_deg
    roll_trim = max(-lim, min(lim, -IMU_ROLL_GAIN * roll_err))
    pitch_trim = max(-lim, min(lim, -IMU_PITCH_GAIN * pitch_deg))
    # roll_trim is negative feedback: if the robot is tipping right, roll the
    # ankles the other way to push back. Opposite signs L/R for the same reason
    # as in gait() — the firmware mirrors this pair, so equal numbers would
    # splay the feet rather than tilt them together.
    j["l_ankle_roll"] += roll_trim
    j["r_ankle_roll"] -= roll_trim
    for side in ("l", "r"):
        j[f"{side}_knee"] += pitch_trim
    return j


class SimpleWalker:
    def __init__(self, host, port, period, use_imu=True, verbose=False):
        self.host, self.port = host, port
        self.period = period
        self.use_imu = use_imu
        self.verbose = verbose
        self._ws = None
        self._running = False
        self._connected = threading.Event()
        self._lock = threading.Lock()
        self._pitch = self._roll = 0.0
        self._last_imu = 0.0
        self._stale_warned = False

    def _on_message(self, ws, msg):
        try:
            imu = json.loads(msg).get("imu", {})
        except Exception:
            return
        with self._lock:
            # Subtract the standing bias so 0 really means upright.
            self._pitch = math.degrees(float(imu.get("pitch", 0.0))) - IMU_PITCH_BIAS
            self._roll = math.degrees(float(imu.get("roll", 0.0))) - IMU_ROLL_BIAS
            self._last_imu = time.time()

    def _on_open(self, ws):
        print("[WS] Connected")
        self._running = True
        self._connected.set()

    def _on_close(self, ws, *_):
        print("[WS] Closed")
        self._running = False

    def _on_error(self, ws, e):
        print(f"[WS] Error: {e}")

    def _send(self, joints_deg):
        angles = {n: math.radians(clamp(n, v)) for n, v in joints_deg.items()}
        try:
            self._ws.send(json.dumps({"cmd": "set_joints", "angles": angles}))
        except Exception as e:
            print(f"[WS] Send error: {e}")

    def run(self, steps):
        url = f"ws://{self.host}:{self.port}"
        print(f"[walk] Connecting to {url}...")
        self._ws = websocket.WebSocketApp(
            url, on_message=self._on_message, on_open=self._on_open,
            on_close=self._on_close, on_error=self._on_error)
        threading.Thread(target=self._ws.run_forever, daemon=True).start()
        if not self._connected.wait(timeout=15.0):
            print("[walk] Could not connect — is firmware-rl running?")
            return

        # Ease into the standing crouch before walking.
        print("[walk] Settling into stance...")
        for i in range(int(1.0 * CTRL_HZ)):
            k = ramp((i + 1) / (1.0 * CTRL_HZ))
            j = {n: 0.0 for n in HW_JOINT_NAMES}
            j["l_knee"] = j["r_knee"] = STANCE_KNEE_DEG * k
            j.update(ARM_LAT)
            self._send(j)
            time.sleep(CTRL_DT)

        total = steps * 0.5 * self.period if steps else float("inf")
        print(f"[walk] Walking — period {self.period}s, shift {SHIFT_DEG}deg. Ctrl+C to stop.")
        t0 = time.time()
        try:
            while self._running:
                t = time.time() - t0
                if t >= total:
                    break
                j = gait((t / self.period) % 1.0)
                with self._lock:
                    p, r, last = self._pitch, self._roll, self._last_imu

                # Frozen telemetry means the ESP32 stopped reporting — usually a
                # servo-current brownout resetting the board or wedging I2C.
                # Keep walking (the firmware has its own timeout) but say so,
                # and stop trusting the stale IMU values.
                stale = last > 0 and (time.time() - last) > 0.5
                if stale and not self._stale_warned:
                    print(f"  [!] IMU telemetry stale at t={t:.1f}s — "
                          f"possible brownout; IMU trim disabled")
                    self._stale_warned = True
                elif not stale:
                    self._stale_warned = False

                if self.use_imu and not stale:
                    # The gait's own commanded lean is the roll we EXPECT to
                    # see, so the trim only fights genuine deviation from it.
                    expected = j["l_hip_roll"] * EXPECTED_ROLL_RATIO
                    j = apply_imu_trim(j, p, r, expected)
                self._send(j)
                if self.verbose and int(t * CTRL_HZ) % 25 == 0:
                    with self._lock:
                        print(f"  t={t:5.2f}  phase={(t/self.period)%1.0:.2f}  "
                              f"roll={self._roll:+5.1f}  pitch={self._pitch:+5.1f}")
                time.sleep(CTRL_DT)
        except KeyboardInterrupt:
            print("\n[walk] Interrupted")
        finally:
            print("[walk] Returning to neutral")
            for i in range(int(0.8 * CTRL_HZ)):
                j = {n: 0.0 for n in HW_JOINT_NAMES}
                j.update(ARM_LAT)
                self._send(j)
                time.sleep(CTRL_DT)
            print("[walk] Done")


def dry_run():
    print(f"period={PERIOD_S}s  shift={SHIFT_DEG}  lift={LIFT_DEG}  step={STEP_DEG}\n")
    names = ["l_hip_roll", "l_hip_pitch", "l_knee", "r_hip_pitch", "r_knee"]
    labels = {0: "SHIFT_L", 1: "SWING_R", 2: "SHIFT_R", 3: "SWING_L"}
    print(f"{'phase':>6} {'quarter':>8} " + " ".join(f"{n:>12}" for n in names))
    for i in range(20):
        ph = i / 20.0
        j = gait(ph)
        q = labels[int(ph * 4) % 4]
        print(f"{ph:6.2f} {q:>8} " + " ".join(f"{j[n]:12.1f}" for n in names))


def measure_bias(host, port, seconds=10.0):
    """Report the IMU's standing offset. Hold the robot upright and still."""
    raw = []

    def on_msg(ws, msg):
        try:
            imu = json.loads(msg).get("imu", {})
        except Exception:
            return
        raw.append((math.degrees(float(imu.get("pitch", 0.0))),
                    math.degrees(float(imu.get("roll", 0.0)))))

    ws = websocket.WebSocketApp(f"ws://{host}:{port}", on_message=on_msg)
    threading.Thread(target=ws.run_forever, daemon=True).start()
    print(f"Hold the robot UPRIGHT and STILL for {seconds:.0f}s...")
    time.sleep(seconds)
    ws.close()
    if not raw:
        print("No telemetry received.")
        return
    pitch = sorted(p for p, _ in raw)
    roll = sorted(r for _, r in raw)
    mp = pitch[len(pitch) // 2]
    mr = roll[len(roll) // 2]
    print(f"samples={len(raw)}")
    print(f"  pitch median {mp:+.2f}  (spread {pitch[-1] - pitch[0]:.2f})")
    print(f"  roll  median {mr:+.2f}  (spread {roll[-1] - roll[0]:.2f})")
    print(f"\nPut these in simple_walk.py:")
    print(f"  IMU_PITCH_BIAS = {mp:.1f}")
    print(f"  IMU_ROLL_BIAS  = {mr:.1f}")


def main():
    p = argparse.ArgumentParser(description="Simple weight-shifting CPG walk")
    p.add_argument("--host", default="optimus-rl.local")
    p.add_argument("--port", type=int, default=81)
    p.add_argument("--period", type=float, default=PERIOD_S)
    p.add_argument("--shift", type=float, help="hip_roll lean amplitude, deg")
    p.add_argument("--lift", type=float, help="swing knee lift, deg")
    p.add_argument("--step", type=float, help="hip_pitch reach, deg")
    p.add_argument("--steps", type=int, default=0, help="0 = walk until Ctrl+C")
    # Open loop is the default. The IMU trim was fighting the gait's deliberate
    # weight shift — it read a commanded lean as a fall and pushed back, so the
    # robot kept starting to lean left and returning to the right. Enable it
    # again with --imu once the open-loop gait is walking on its own.
    p.add_argument("--imu", action="store_true",
                   help="enable IMU trim (off by default)")
    p.add_argument("--no-imu", action="store_true",
                   help=argparse.SUPPRESS)   # kept so old commands still work
    p.add_argument("--dry-run", action="store_true")
    p.add_argument("--measure-bias", action="store_true",
                   help="hold the robot upright and still; reports the IMU "
                        "standing bias to put in IMU_ROLL/PITCH_BIAS")
    p.add_argument("--verbose", action="store_true")
    a = p.parse_args()

    if a.measure_bias:
        measure_bias(a.host, a.port)
        return

    global SHIFT_DEG, LIFT_DEG, STEP_DEG
    if a.shift is not None:
        SHIFT_DEG = a.shift
    if a.lift is not None:
        LIFT_DEG = a.lift
    if a.step is not None:
        STEP_DEG = a.step

    if a.dry_run:
        dry_run()
        return
    SimpleWalker(a.host, a.port, a.period,
                 use_imu=a.imu and not a.no_imu,
                 verbose=a.verbose).run(a.steps)


if __name__ == "__main__":
    main()
