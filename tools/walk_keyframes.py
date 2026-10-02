"""
walk_keyframes.py — Optimus walk built from hand-tuned keyframes.

Every pose in KEYFRAMES was verified on the real robot, one at a time, with a
human watching and correcting the signs. That is why this file does not try to
derive anything: the previous CPG attempts kept guessing joint directions from
config.h and docs/JOINTS.md, and both turned out to disagree with the hardware
in several places (hip_roll mirroring, ankle direction, knee sign per leg).

Half a gait cycle, right leg supporting first:

    0 NEUTRAL    stand tall, both legs straight
    1 SHIFT_R    tip the body onto the RIGHT foot
    2 LIFT_L     left hip inward, knee up, ankle rolled to break friction
    3 REACH_L    swing the left leg forward (upper joint leads)
    4 PUSH_R     right lower knee drives back — this is the propulsion
    5 TRANSFER   weight crosses onto the left leg
    6 PLANT_L    left leg extends straight, right flexes to clear, ankles freed

The second half mirrors these with the legs swapped (MIRROR below).

Usage:
    python walk_keyframes.py --dry-run        # print the pose table
    python walk_keyframes.py --half           # one half cycle then stop
    python walk_keyframes.py --step-time 0.8  # slower per keyframe
    python walk_keyframes.py --cycles 4
"""

import argparse
import json
import math
import threading
import time

import websocket

HW = ["l_hip_roll", "l_hip_pitch", "l_knee", "l_ankle_roll",
      "r_hip_roll", "r_hip_pitch", "r_knee", "r_ankle_roll",
      "l_shoulder_fb", "r_shoulder_fb", "l_shoulder_lat", "r_shoulder_lat",
      "l_forearm_lat", "r_forearm_lat", "hip_yaw"]

LIMITS = {"hip_roll": 45, "hip_pitch": 90, "knee": 70, "ankle_roll": 90,
          "shoulder_fb": 120, "shoulder_lat": 180, "forearm_lat": 90,
          "hip_yaw": 45}

ARM_LAT = {"l_shoulder_lat": -90.0, "r_shoulder_lat": 90.0}

# ── Keyframes, degrees. Only non-zero joints are listed. ─────────────────────
# Verified pose by pose on hardware 2026-08-15.
KEYFRAMES = [
    ("NEUTRAL",  {}),
    ("SHIFT_R",  {"l_hip_roll": -22, "r_hip_roll": 22}),
    ("LIFT_L",   {"l_hip_roll": 0, "r_hip_roll": 22,
                  "l_knee": 35, "l_ankle_roll": 45}),
    ("REACH_L",  {"l_hip_roll": 0, "r_hip_roll": 22,
                  "l_ankle_roll": 45, "l_hip_pitch": 80, "l_knee": 20}),
    ("PUSH_R",   {"l_hip_roll": -20, "r_hip_roll": 42,
                  "l_ankle_roll": 45, "l_hip_pitch": 80, "l_knee": 20,
                  "r_knee": 50}),
    # The outgoing leg's ankle has to PUSH toward the incoming leg here. Left
    # at 0 it stayed passive exactly while the weight was crossing, and the
    # robot toppled back onto the side it was trying to leave.
    ("TRANSFER", {"l_hip_roll": -3, "r_hip_roll": 20,
                  "l_hip_pitch": 80, "l_knee": 20, "r_knee": 50,
                  "r_ankle_roll": 25}),
    # The outgoing ankle keeps pushing here, harder than during TRANSFER (+35
    # vs +25). This is the moment both legs straighten again, and letting that
    # ankle relax — it used to invert to -30 — dropped the robot back toward
    # the side it had just left, right as the new stance leg extended.
    ("PLANT_L",  {"l_hip_roll": 17, "r_hip_roll": 0,
                  "l_ankle_roll": -30, "r_ankle_roll": 35, "r_knee": 25}),

    # ── Second half: left supports, right steps ─────────────────────────────
    # Measured on hardware rather than mirrored. The automatic mirror assumed
    # the legs are symmetric, and they are not: the right hip needs 105 deg to
    # reach as far forward as the left does at 80.
    ("LIFT_R",   {"l_hip_roll": 17, "r_hip_roll": 0,
                  "l_ankle_roll": -30, "r_ankle_roll": 35, "r_knee": 45}),
    ("REACH_R",  {"l_hip_roll": 17, "r_hip_roll": 0,
                  "l_ankle_roll": -30, "r_ankle_roll": 35,
                  "r_knee": 20, "r_hip_pitch": 105}),
    ("PUSH_L",   {"l_hip_roll": -3, "r_hip_roll": 0,
                  "l_ankle_roll": -30, "r_ankle_roll": 35,
                  "r_knee": 20, "r_hip_pitch": 105, "l_knee": 50}),
    # Left hip swings hard inward (+45, at the firmware clamp) to drive the
    # body across, while the left ankle pushes (+25) — the same push that the
    # first half needed, mirrored onto the other foot.
    # l_ankle_roll 25 -> 15, same reason as PLANT_R below: the push toward the
    # right was strong enough to nearly tip the robot (roll reached +25 here).
    # l_hip_roll 45 -> 30: at the clamp the two hips pulled the legs so far in
    # that the feet collided here and the robot went down.
    ("TRANSFER_R", {"l_hip_roll": 10, "r_hip_roll": 3,
                    "l_ankle_roll": 5, "r_hip_pitch": 105,
                    "r_knee": 20, "l_knee": 50}),
    # l_ankle_roll 25 -> 15: the outgoing ankle was pushing so far right that
    # the robot was on the edge of toppling at this frame (roll hit +27).
    ("PLANT_R",  {"l_hip_roll": 30, "r_hip_roll": -17,
                  "l_ankle_roll": 5, "r_ankle_roll": -20, "l_knee": 50}),

    # ── Wrap-around ─────────────────────────────────────────────────────────
    # Without this the cycle does not close on itself: PLANT_R ends with the
    # left hip fully inward (+45) and its knee flexed, while SHIFT_R expects a
    # robot standing square. Going straight between them moved 201 deg of total
    # joint travel in one blend — against ~20 deg for a normal step — so the
    # robot stopped and restarted instead of walking continuously.
    # This pose opens the left leg back up while the weight stays right.
    ("RECOVER",  {"l_hip_roll": 10, "r_hip_roll": 0,
                  "l_ankle_roll": 10, "r_ankle_roll": -10, "l_knee": 25}),
]

# Left/right joint pairs, for generating the mirrored half cycle.
MIRROR = {
    "l_hip_roll": "r_hip_roll", "l_hip_pitch": "r_hip_pitch",
    "l_knee": "r_knee", "l_ankle_roll": "r_ankle_roll",
    "l_shoulder_fb": "r_shoulder_fb",
}

CTRL_HZ = 50.0
CTRL_DT = 1.0 / CTRL_HZ


def clamp(name, v):
    for key, lim in LIMITS.items():
        if name.endswith(key):
            return max(-lim, min(lim, v))
    return v


def full_pose(partial):
    """Expand a keyframe into every joint, with arms at their rest position."""
    p = {n: 0.0 for n in HW}
    p.update(ARM_LAT)
    p.update(partial)
    return p


def mirrored(partial):
    """
    Swap left and right joints to produce the opposite half cycle.

    The roll axes also flip sign: leaning onto the right foot is the negative
    of leaning onto the left. Pitch and knee keep their value because each side
    already has its own sign convention baked into the tuned numbers.
    """
    # Roll joints swap sides AND flip sign: SHIFT_R leans onto the right foot
    # with (l=-22, r=+22), so its mirror must lean onto the left with
    # (l=+22, r=-22) — swapping alone produced (l=+22, r=-22) read as leaning
    # further onto the left leg that was already loaded, so the right foot
    # never unloaded and the second half of the cycle never took a step.
    # Pitch and knee only swap: each side's tuned numbers already encode its
    # own direction convention.
    out = {}
    for name, val in partial.items():
        tgt = MIRROR.get(name)
        if tgt is None and name in MIRROR.values():
            tgt = [k for k, v in MIRROR.items() if v == name][0]
        if tgt is None:
            out[name] = val
        else:
            out[tgt] = val
    return out


def build_cycle():
    """
    Full cycle. KEYFRAMES now holds BOTH halves as measured on the robot, so
    nothing is mirrored: the two legs turned out not to be symmetric (the right
    hip needs 105 deg of pitch where the left needs 80), and the generated
    mirror produced a second half that never unloaded the right foot.
    NEUTRAL is only used to enter and leave the gait, not every cycle.
    """
    # NEUTRAL and SHIFT_R are entry poses only.
    #
    # NEUTRAL would stand the robot up between laps. SHIFT_R exists to tip a
    # squarely-standing robot onto its right foot, but once RECOVER has already
    # left the weight on the right, repeating it drives the left hip out to -22
    # while straightening that knee — the left leg slides out sideways instead
    # of lifting into its next step.
    # LIFT_L is dropped for the same reason. It raised the left knee to 35 only
    # for REACH_L to drop it back to 20 one frame later — a lift that undoes
    # itself. It was needed when the cycle still came from SHIFT_R with that leg
    # straight, but RECOVER already leaves it flexed at 25. Skipping it also
    # shortens the transition (162 deg of travel instead of 182).
    skip = {"NEUTRAL", "SHIFT_R", "LIFT_L"}
    return [(n, full_pose(p)) for n, p in KEYFRAMES if n not in skip]


def ease(t):
    """Smooth 0->1, zero slope at both ends — no step inputs to the servos."""
    return 0.5 - 0.5 * math.cos(math.pi * max(0.0, min(1.0, t)))


class Walker:
    def __init__(self, host, port, step_time, verbose=False):
        self.host, self.port = host, port
        self.step_time = step_time
        self.verbose = verbose
        self._ws = None
        self._connected = threading.Event()
        self._running = False
        self._lock = threading.Lock()
        self._roll = self._pitch = 0.0

    def _on_msg(self, ws, m):
        try:
            imu = json.loads(m).get("imu", {})
        except Exception:
            return
        with self._lock:
            self._roll = math.degrees(float(imu.get("roll", 0.0))) - 7.3
            self._pitch = math.degrees(float(imu.get("pitch", 0.0))) - 7.2

    def _on_open(self, ws):
        self._running = True
        self._connected.set()
        print("[WS] connected")

    def _on_close(self, ws, *_):
        self._running = False

    def _send(self, pose):
        try:
            self._ws.send(json.dumps({"cmd": "set_joints", "angles": {
                k: math.radians(clamp(k, v)) for k, v in pose.items()}}))
        except Exception as e:
            print(f"[WS] send error: {e}")

    def _blend(self, a, b, name=""):
        steps = max(1, int(self.step_time * CTRL_HZ))
        for i in range(steps):
            k = ease((i + 1) / steps)
            self._send({j: a[j] + (b[j] - a[j]) * k for j in a})
            time.sleep(CTRL_DT)
        if self.verbose:
            with self._lock:
                print(f"  {name:10} roll {self._roll:+6.1f}  pitch {self._pitch:+6.1f}")

    def run(self, cycles, half=False):
        url = f"ws://{self.host}:{self.port}"
        print(f"[walk] connecting to {url}...")
        self._ws = websocket.WebSocketApp(
            url, on_message=self._on_msg, on_open=self._on_open,
            on_close=self._on_close)
        threading.Thread(target=self._ws.run_forever, daemon=True).start()
        if not self._connected.wait(timeout=15.0):
            print("[walk] no connection — is firmware-rl running?")
            return

        frames = build_cycle()
        if half:
            frames = frames[:len(KEYFRAMES)]
        cur = full_pose({})
        self._blend(cur, cur, "settle")
        time.sleep(0.3)

        # Enter the gait: tip onto the right foot once. SHIFT_R is not part of
        # the repeating cycle (see build_cycle), so it runs only here.
        entry = dict(KEYFRAMES)["SHIFT_R"]
        self._blend(cur, full_pose(entry), "SHIFT_R")
        cur = full_pose(entry)

        print(f"[walk] {len(frames)} keyframes, {self.step_time}s each. Ctrl+C to stop.")
        try:
            for c in range(cycles):
                if cycles > 1:
                    print(f"[cycle {c + 1}/{cycles}]")
                for name, pose in frames:
                    self._blend(cur, pose, name)
                    cur = pose
        except KeyboardInterrupt:
            print("\n[walk] interrupted")
        finally:
            print("[walk] returning to neutral")
            self._blend(cur, full_pose({}), "neutral")
            print("[walk] done")


def dry_run():
    frames = build_cycle()
    cols = ["l_hip_roll", "l_hip_pitch", "l_knee", "l_ankle_roll",
            "r_hip_roll", "r_hip_pitch", "r_knee", "r_ankle_roll"]
    print(f"{'frame':10} " + " ".join(f"{c.replace('_roll','R').replace('_pitch','P'):>9}"
                                       for c in cols))
    for name, p in frames:
        print(f"{name:10} " + " ".join(f"{p[c]:9.0f}" for c in cols))


def main():
    ap = argparse.ArgumentParser(description="Keyframe walk for Optimus")
    ap.add_argument("--host", default="optimus-rl.local")
    ap.add_argument("--port", type=int, default=81)
    ap.add_argument("--step-time", type=float, default=0.6,
                    help="seconds to blend between keyframes (default 0.6)")
    ap.add_argument("--cycles", type=int, default=3)
    ap.add_argument("--half", action="store_true",
                    help="only the tuned half cycle, no mirror")
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--verbose", action="store_true")
    a = ap.parse_args()

    if a.dry_run:
        dry_run()
        return
    Walker(a.host, a.port, a.step_time, a.verbose).run(a.cycles, a.half)


if __name__ == "__main__":
    main()
