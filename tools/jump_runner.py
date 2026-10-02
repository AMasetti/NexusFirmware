"""
jump_runner.py — squat and jump controller for Optimus (firmware-rl).

Drives the ESP32 over the same WebSocket protocol as rl_runner.py, but instead
of a walking policy it runs a scripted five-phase jump:

    SETTLE   hold the neutral standing pose, let the servos catch up
    SQUAT    ease down into a crouch over SQUAT_S seconds
    CHARGE   hold the crouch briefly so the legs stop moving before launch
    LAUNCH   extend explosively — this is the only phase that tries to leave
             the ground, and it is deliberately short
    LAND     return to a soft crouch to absorb impact, then back to neutral

Usage:
    python jump_runner.py                       # one jump, default depth
    python jump_runner.py --squat-only          # crouch and hold (no launch)
    python jump_runner.py --depth 0.7 --jumps 3
    python jump_runner.py --dry-run             # print angles, send nothing

Safety:
    - --squat-only lets you verify the crouch pose before any explosive motion.
    - Ctrl+C returns the robot to neutral instead of freezing mid-pose.
    - Angles are clamped to the same joint limits the firmware enforces, so a
      bad --depth cannot command past the servo range.

NOTE ON EXPECTATIONS: MG995 servos are unlikely to produce enough peak torque
to actually lift this robot off the ground. Treat a successful "jump" as the
extension happening cleanly and fast; real air time would be a bonus. The
LAUNCH phase is what stresses the servos most — see --depth to soften it.
"""

import argparse
import json
import math
import os
import sys
import threading
import time

import websocket  # pip install websocket-client

# ── Joint set (must match firmware-rl WebSocket field names) ──────────────────
HW_JOINT_NAMES = [
    "l_hip_roll", "l_hip_pitch", "l_knee", "l_ankle_roll",
    "r_hip_roll", "r_hip_pitch", "r_knee", "r_ankle_roll",
    "l_shoulder_fb", "r_shoulder_fb", "l_shoulder_lat", "r_shoulder_lat",
    "l_forearm_lat", "r_forearm_lat", "hip_yaw",
]

# Firmware soft limits (optimus/firmware-rl/include/config.h), radians.
# Commands are clamped to these so a bad depth cannot drive past the servo range.
LIMITS = {
    "l_hip_roll":     math.radians(45),  "r_hip_roll":     math.radians(45),
    "l_hip_pitch":    math.radians(60),  "r_hip_pitch":    math.radians(60),
    "l_knee":         math.radians(70),  "r_knee":         math.radians(70),
    "l_ankle_roll":   math.radians(90),  "r_ankle_roll":   math.radians(90),
    "l_shoulder_fb":  math.radians(120), "r_shoulder_fb":  math.radians(120),
    "l_shoulder_lat": math.radians(180), "r_shoulder_lat": math.radians(180),
    "l_forearm_lat":  math.radians(90),  "r_forearm_lat":  math.radians(90),
    "hip_yaw":        math.radians(45),
}

# Arms hang at the sides during the jump (firmware halt pose is a T-pose).
ARM_HOLD = {
    "l_shoulder_lat": math.radians(-90.0),
    "r_shoulder_lat": math.radians(+90.0),
}

# shoulder_fb direction per side. The right shoulder servo is mounted mirrored,
# so it needs the opposite sign to swing the same way as the left. Flip these
# if the arms still move in opposite directions.
ARM_FB_SIGN = {"l": +1.0, "r": -1.0}

# ── Pose definitions ──────────────────────────────────────────────────────────
# Squat depth is expressed as a fraction: 1.0 = the deepest crouch below.
# hip_pitch and knee move together; the ankle counter-rotates to keep the sole
# flat on the ground, which is what lets the legs load evenly.
SQUAT_HIP_PITCH = math.radians(45.0)
SQUAT_KNEE      = math.radians(65.0)
SQUAT_ANKLE     = math.radians(25.0)

# Launch target: fully straight, no overshoot. Driving past 0 does not add
# impulse — the leg is already at its mechanical stop there, so the extra
# command just stalls the servo against the end of travel and wastes the
# stroke. The push comes from covering the whole squat->straight sweep fast.
LAUNCH_HIP_PITCH = math.radians(0.0)
LAUNCH_KNEE      = math.radians(0.0)
# The ankle is the exception: it keeps pushing after the knee is straight, so
# a few degrees of plantarflexion here adds a real toe-off at the end.
LAUNCH_ANKLE     = math.radians(-15.0)

# Phase durations, seconds.
SETTLE_S = 1.0
# A countermovement jump gets much of its impulse from dropping fast and
# reversing without pausing — the downward momentum preloads the legs. The
# original 1.2 s descent + 0.35 s hold threw all of that away and launched
# from a dead stop, which is why it felt so weak.
SQUAT_S  = 0.35
CHARGE_S = 0.0
# LAUNCH_S was 0.12 s and produced no push at all: an MG995 needs ~0.17 s per
# 60 deg unloaded, and the extension sweeps ~75 deg of knee under full body
# load. The phase ended before the servo had travelled even half way, so the
# command was replaced by the landing pose mid-stroke. 0.25 s lets the stroke
# actually complete — that is what produces the impulse.
LAUNCH_S = 0.25
LAND_S   = 0.9

CTRL_HZ = 50.0
CTRL_DT = 1.0 / CTRL_HZ


def clamp(name: str, value: float) -> float:
    lim = LIMITS.get(name)
    if lim is None:
        return value
    return max(-lim, min(lim, value))


def leg_pose(hip_pitch: float, knee: float, ankle: float,
             arm_swing: float = 0.0) -> dict:
    """
    Symmetric leg pose.

    arm_swing: shoulder_fb angle in radians. Swinging the arms up during the
    launch adds real upward momentum — on a robot this light the arms are a
    meaningful fraction of total mass, so throwing them is not cosmetic.
    """
    pose = {n: 0.0 for n in HW_JOINT_NAMES}
    for side in ("l", "r"):
        pose[f"{side}_hip_pitch"]  = hip_pitch
        pose[f"{side}_knee"]       = knee
        pose[f"{side}_ankle_roll"] = ankle
        # shoulder_fb is mirrored L/R on this robot: the same angle drives one
        # arm forward and the other back, so they cancel instead of adding.
        # Observed on hardware — left went forward, right went back.
        pose[f"{side}_shoulder_fb"] = arm_swing * ARM_FB_SIGN[side]
    pose.update(ARM_HOLD)
    return {k: clamp(k, v) for k, v in pose.items()}


# Arms wind down during the squat, then throw up through the launch.
ARM_WIND  = math.radians(-35.0)   # back/down while crouching
ARM_THROW = math.radians(+75.0)   # up and forward at take-off


def neutral_pose() -> dict:
    return leg_pose(0.0, 0.0, 0.0)


def squat_pose(depth: float) -> dict:
    return leg_pose(SQUAT_HIP_PITCH * depth,
                    SQUAT_KNEE * depth,
                    SQUAT_ANKLE * depth,
                    arm_swing=ARM_WIND * depth)


def launch_pose() -> dict:
    return leg_pose(LAUNCH_HIP_PITCH, LAUNCH_KNEE, LAUNCH_ANKLE,
                    arm_swing=ARM_THROW)


def lerp_pose(a: dict, b: dict, t: float) -> dict:
    t = max(0.0, min(1.0, t))
    return {k: a[k] + (b[k] - a[k]) * t for k in a}


def ease_in_out(t: float) -> float:
    """Smooth ramp — avoids the step input that makes these servos buzz."""
    return 0.5 - 0.5 * math.cos(math.pi * max(0.0, min(1.0, t)))


class JumpRunner:
    def __init__(self, host: str, port: int, depth: float,
                 dry_run: bool = False, verbose: bool = False):
        self._host = host
        self._port = port
        self._depth = depth
        self._dry = dry_run
        self._verbose = verbose

        self._ws = None
        self._running = False
        self._connected = threading.Event()
        self._lock = threading.Lock()
        self._pitch = 0.0
        self._roll = 0.0
        self._peak_pitch = 0.0

    # ── WebSocket ────────────────────────────────────────────────────────────

    def _on_message(self, ws, message):
        try:
            d = json.loads(message)
        except Exception:
            return
        imu = d.get("imu", {})
        with self._lock:
            self._pitch = float(imu.get("pitch", 0.0))
            self._roll = float(imu.get("roll", 0.0))
            self._peak_pitch = max(self._peak_pitch, abs(self._pitch))

    def _on_open(self, ws):
        print("[WS] Connected to ESP32")
        self._running = True
        self._connected.set()

    def _on_close(self, ws, *_):
        print("[WS] Connection closed")
        self._running = False

    def _on_error(self, ws, error):
        print(f"[WS] Error: {error}")

    def _send(self, angles: dict):
        if self._dry:
            return
        try:
            self._ws.send(json.dumps({"cmd": "set_joints", "angles": angles}))
        except Exception as e:
            print(f"[WS] Send error: {e}")

    # ── Motion ───────────────────────────────────────────────────────────────

    def _run_phase(self, name: str, start: dict, end: dict,
                   duration: float, ease: bool = True):
        """Interpolate start -> end over duration at CTRL_HZ."""
        steps = max(1, int(duration * CTRL_HZ))
        if self._verbose:
            print(f"  [{name}] {duration:.2f}s ({steps} steps)")
        for i in range(steps):
            if not self._dry and not self._running:
                raise RuntimeError("connection lost mid-phase")
            t = (i + 1) / steps
            pose = lerp_pose(start, end, ease_in_out(t) if ease else t)
            self._send(pose)
            time.sleep(CTRL_DT)
        return end

    def _hold(self, pose: dict, duration: float, name: str = "hold"):
        steps = max(1, int(duration * CTRL_HZ))
        if self._verbose:
            print(f"  [{name}] {duration:.2f}s")
        for _ in range(steps):
            self._send(pose)
            time.sleep(CTRL_DT)
        return pose

    def jump_once(self, squat_only: bool = False):
        neutral = neutral_pose()
        squat = squat_pose(self._depth)

        self._hold(neutral, SETTLE_S, "SETTLE")

        if squat_only:
            # Gentle descent when we are only inspecting the pose.
            self._run_phase("SQUAT", neutral, squat, 1.2)
            print("  [SQUAT-ONLY] holding crouch — Ctrl+C to release")
            while self._dry or self._running:
                self._send(squat)
                time.sleep(CTRL_DT)
            return

        # Fast drop, no easing at the bottom: the leg should still be moving
        # down when the launch reverses it. Easing here decelerates into the
        # crouch and kills the countermovement.
        self._run_phase("DROP", neutral, squat, SQUAT_S, ease=False)

        if CHARGE_S > 0:
            self._hold(squat, CHARGE_S, "CHARGE")

        with self._lock:
            self._peak_pitch = 0.0

        # LAUNCH: no easing — we want the fastest command ramp the servos will take.
        self._run_phase("LAUNCH", squat, launch_pose(), LAUNCH_S, ease=False)

        # LAND: soft crouch to absorb, then stand.
        landing = squat_pose(self._depth * 0.5)
        self._run_phase("LAND", launch_pose(), landing, LAND_S * 0.4)
        self._run_phase("RECOVER", landing, neutral, LAND_S * 0.6)

        with self._lock:
            peak = math.degrees(self._peak_pitch)
        print(f"  peak |pitch| during launch/landing: {peak:.1f} deg")

    # ── Entry point ──────────────────────────────────────────────────────────

    def run(self, jumps: int, squat_only: bool):
        if self._dry:
            print("[runner] DRY RUN — no packets will be sent")
            self._dry_preview(squat_only)
            return

        url = f"ws://{self._host}:{self._port}"
        print(f"[runner] Connecting to {url}...")
        self._ws = websocket.WebSocketApp(
            url,
            on_message=self._on_message,
            on_open=self._on_open,
            on_close=self._on_close,
            on_error=self._on_error,
        )
        threading.Thread(target=self._ws.run_forever, daemon=True).start()

        if not self._connected.wait(timeout=15.0):
            print("[runner] Could not connect — is firmware-rl running?")
            return

        time.sleep(0.3)  # let the first telemetry packets arrive
        try:
            for i in range(jumps):
                if jumps > 1:
                    print(f"[jump {i + 1}/{jumps}]")
                self.jump_once(squat_only=squat_only)
                if i + 1 < jumps:
                    self._hold(neutral_pose(), 1.0, "PAUSE")
        except KeyboardInterrupt:
            print("\n[runner] Interrupted — returning to neutral")
        except RuntimeError as e:
            print(f"[runner] {e}")
        finally:
            # Always leave the robot standing rather than frozen mid-pose.
            try:
                self._hold(neutral_pose(), 0.6, "NEUTRAL")
            except Exception:
                pass
            print("[runner] Done")

    def _dry_preview(self, squat_only: bool):
        import itertools
        names = ["l_hip_pitch", "l_knee", "l_ankle_roll"]
        rows = [("neutral", neutral_pose()),
                ("squat", squat_pose(self._depth))]
        if not squat_only:
            rows.append(("launch", launch_pose()))
            rows.append(("landing", squat_pose(self._depth * 0.5)))
        w = max(len(n) for n, _ in rows)
        print(f"{'pose':<{w}} " + " ".join(f"{n:>14}" for n in names))
        for label, pose in rows:
            print(f"{label:<{w}} " +
                  " ".join(f"{math.degrees(pose[n]):>13.1f}°" for n in names))


def main():
    global LAUNCH_S
    p = argparse.ArgumentParser(description="Squat and jump controller for Optimus")
    p.add_argument("--host", default="optimus-rl.local")
    p.add_argument("--port", type=int, default=81)
    p.add_argument("--depth", type=float, default=1.0,
                   help="squat depth fraction, 0.3-1.0 (default 1.0 = full crouch)")
    p.add_argument("--jumps", type=int, default=1)
    p.add_argument("--launch-s", type=float, default=LAUNCH_S,
                   help=f"launch stroke duration in seconds (default {LAUNCH_S}). "
                        "Too short and the servo never completes the stroke; too "
                        "long and it becomes a slow stand-up with no impulse.")
    p.add_argument("--squat-only", action="store_true",
                   help="crouch and hold; never launch (verify the pose first)")
    p.add_argument("--dry-run", action="store_true",
                   help="print the pose table and exit without connecting")
    p.add_argument("--verbose", action="store_true")
    a = p.parse_args()

    if not 0.2 <= a.depth <= 1.0:
        p.error("--depth must be between 0.2 and 1.0")

    LAUNCH_S = a.launch_s

    JumpRunner(a.host, a.port, a.depth,
               dry_run=a.dry_run, verbose=a.verbose).run(a.jumps, a.squat_only)


if __name__ == "__main__":
    main()
