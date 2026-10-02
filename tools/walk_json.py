"""
walk_json.py — run the Optimus walk from walk_poses.json.

Edit the JSON, run this. No Python editing, no redeploy.

    python optimus/tools/walk_json.py                  # walk the cycle
    python optimus/tools/walk_json.py --table          # print poses, send nothing
    python optimus/tools/walk_json.py --pose REACH_L   # hold one pose to inspect
    python optimus/tools/walk_json.py --cycles 1 --step-time 1.2
    python optimus/tools/walk_json.py --from PLANT_L   # start mid-cycle
    python optimus/tools/walk_json.py --watch          # re-read JSON every cycle

JSON layout:
    step_time  seconds to blend between poses
    cycles     how many laps (0 = until Ctrl+C)
    entry      poses played once, to get into the gait
    cycle      poses repeated, in order
    poses      name -> {joint: degrees}

Any joint left out of a pose is 0. Arms are held at their rest position
automatically. Values are clamped to the firmware limits before sending, and
--table flags anything that would be clipped.
"""

import argparse
import json
import math
import os
import sys
import threading
import time

import websocket

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_JSON = os.path.join(HERE, "walk_poses.json")

HW = ["l_hip_roll", "l_hip_pitch", "l_knee", "l_ankle_roll",
      "r_hip_roll", "r_hip_pitch", "r_knee", "r_ankle_roll",
      "l_shoulder_fb", "r_shoulder_fb", "l_shoulder_lat", "r_shoulder_lat",
      "l_forearm_lat", "r_forearm_lat", "hip_yaw"]

LIMITS = {"hip_roll": 45, "hip_pitch": 90, "knee": 70, "ankle_roll": 90,
          "shoulder_fb": 120, "shoulder_lat": 180, "forearm_lat": 90,
          "hip_yaw": 45}

ARM_LAT = {"l_shoulder_lat": -90.0, "r_shoulder_lat": 90.0}

# IMU standing offset, subtracted so 0 means upright (measured 2026-08-15).
IMU_ROLL_BIAS, IMU_PITCH_BIAS = 7.3, 7.2

CTRL_HZ = 50.0
CTRL_DT = 1.0 / CTRL_HZ

TABLE_COLS = ["l_hip_roll", "l_hip_pitch", "l_knee", "l_ankle_roll",
              "r_hip_roll", "r_hip_pitch", "r_knee", "r_ankle_roll",
              "l_shoulder_fb", "r_shoulder_fb",
              "l_forearm_lat", "r_forearm_lat", "hip_yaw",
              "l_shoulder_lat", "r_shoulder_lat"]


def limit_for(name):
    for key, lim in LIMITS.items():
        if name.endswith(key):
            return lim
    return None


def clamp(name, v):
    lim = limit_for(name)
    return v if lim is None else max(-lim, min(lim, v))


def load(path):
    with open(path) as f:
        cfg = json.load(f)
    for key in ("poses", "cycle"):
        if key not in cfg:
            sys.exit(f"{path}: missing '{key}'")
    missing = [n for n in cfg["cycle"] + cfg.get("entry", [])
               if n not in cfg["poses"]]
    if missing:
        sys.exit(f"{path}: cycle/entry names not found in poses: {missing}")
    return cfg


def full_pose(partial):
    p = {n: 0.0 for n in HW}
    p.update(ARM_LAT)
    p.update({k: float(v) for k, v in partial.items() if k in p})
    return p


def ease(t):
    """Smooth 0->1 with zero slope at both ends — servos hate step inputs."""
    return 0.5 - 0.5 * math.cos(math.pi * max(0.0, min(1.0, t)))


def print_table(cfg):
    hdr = {"l_hip_roll": "l_hipR", "l_hip_pitch": "l_hipP", "l_knee": "l_knee",
           "l_ankle_roll": "l_ankR", "r_hip_roll": "r_hipR",
           "r_hip_pitch": "r_hipP", "r_knee": "r_knee", "r_ankle_roll": "r_ankR",
           "l_shoulder_fb": "l_armFB", "r_shoulder_fb": "r_armFB",
           "l_forearm_lat": "l_fore", "r_forearm_lat": "r_fore",
           "hip_yaw": "hipYaw",
           "l_shoulder_lat": "l_abd", "r_shoulder_lat": "r_abd"}
    print(f"{'frame':12} " + " ".join(f"{hdr[c]:>8}" for c in TABLE_COLS))
    names = cfg.get("entry", []) + cfg["cycle"]
    seen = []
    for n in names:
        if n in seen:
            continue
        seen.append(n)
        p = cfg["poses"][n]
        cells = []
        for c in TABLE_COLS:
            v = float(p.get(c, 0.0))
            lim = limit_for(c)
            flag = "!" if lim and abs(v) > lim else " "
            cells.append(f"{v:7.0f}{flag}")
        tag = "" if n in cfg["cycle"] else "  (entry only)"
        print(f"{n:12} " + " ".join(cells) + tag)

    print()
    prev = None
    # Leg travel only — arm swing would inflate these and hide leg jolts.
    leg_cols = [c for c in TABLE_COLS if "shoulder" not in c]
    print("leg travel between consecutive cycle poses:")
    for n in cfg["cycle"] + [cfg["cycle"][0]]:
        cur = full_pose(cfg["poses"][n])
        if prev is not None:
            tot = sum(abs(cur[k] - prev[1][k]) for k in leg_cols)
            same = tot < 1.0
            print(f"  {prev[0]:12} -> {n:12} {tot:6.0f}"
                  + ("   IDENTICAL" if same else ""))
        prev = (n, cur)
    print("  (last line is the wrap-around back to the first pose)")


class Runner:
    def __init__(self, host, port, verbose=False):
        self.host, self.port = host, port
        self.verbose = verbose
        self._ws = None
        self._connected = threading.Event()
        self._lock = threading.Lock()
        self._roll = self._pitch = 0.0

    def _on_msg(self, ws, m):
        try:
            imu = json.loads(m).get("imu", {})
        except Exception:
            return
        with self._lock:
            self._roll = math.degrees(float(imu.get("roll", 0.0))) - IMU_ROLL_BIAS
            self._pitch = math.degrees(float(imu.get("pitch", 0.0))) - IMU_PITCH_BIAS

    def _on_open(self, ws):
        self._connected.set()
        print("[ws] connected")

    def connect(self):
        url = f"ws://{self.host}:{self.port}"
        print(f"[ws] connecting to {url} ...")
        self._ws = websocket.WebSocketApp(url, on_message=self._on_msg,
                                          on_open=self._on_open)
        threading.Thread(target=self._ws.run_forever, daemon=True).start()
        if not self._connected.wait(timeout=15.0):
            sys.exit("could not connect — is firmware-rl running?")
        time.sleep(0.3)

    def send(self, pose):
        try:
            self._ws.send(json.dumps({"cmd": "set_joints", "angles": {
                k: math.radians(clamp(k, v)) for k, v in pose.items()}}))
        except Exception as e:
            print(f"[ws] send error: {e}")

    def blend(self, a, b, seconds, label=""):
        steps = max(1, int(seconds * CTRL_HZ))
        for i in range(steps):
            k = ease((i + 1) / steps)
            self.send({j: a[j] + (b[j] - a[j]) * k for j in a})
            time.sleep(CTRL_DT)
        if self.verbose:
            with self._lock:
                print(f"  {label:12} roll {self._roll:+6.1f}  pitch {self._pitch:+6.1f}")

    def hold(self, pose, seconds, label=""):
        for _ in range(max(1, int(seconds * CTRL_HZ))):
            self.send(pose)
            time.sleep(CTRL_DT)
        with self._lock:
            print(f"  {label:12} roll {self._roll:+6.1f}  pitch {self._pitch:+6.1f}")


def main():
    ap = argparse.ArgumentParser(description="Run the walk from walk_poses.json")
    ap.add_argument("--json", default=DEFAULT_JSON)
    ap.add_argument("--host", default="optimus-rl.local")
    ap.add_argument("--port", type=int, default=81)
    ap.add_argument("--step-time", type=float)
    ap.add_argument("--cycles", type=int)
    ap.add_argument("--pose", help="hold one pose and report the IMU")
    ap.add_argument("--from", dest="start", help="start the cycle at this pose")
    ap.add_argument("--no-entry", action="store_true",
                    help="skip the entry poses")
    ap.add_argument("--watch", action="store_true",
                    help="re-read the JSON before every cycle")
    ap.add_argument("--table", action="store_true",
                    help="print the poses and exit")
    ap.add_argument("--verbose", action="store_true", default=True)
    a = ap.parse_args()

    cfg = load(a.json)
    if a.table:
        print_table(cfg)
        return

    step_time = a.step_time or cfg.get("step_time", 0.8)
    cycles = a.cycles if a.cycles is not None else cfg.get("cycles", 3)

    r = Runner(a.host, a.port, a.verbose)
    r.connect()

    if a.pose:
        if a.pose not in cfg["poses"]:
            sys.exit(f"no such pose: {a.pose}")
        print(f"holding {a.pose} — Ctrl+C to stop")
        try:
            while True:
                r.hold(full_pose(cfg["poses"][a.pose]), 2.0, a.pose)
        except KeyboardInterrupt:
            print("\nreturning to neutral")
            r.blend(full_pose(cfg["poses"][a.pose]), full_pose({}), 0.8)
        return

    cur = full_pose({})
    try:
        if not a.no_entry:
            for n in cfg.get("entry", []):
                r.blend(cur, full_pose(cfg["poses"][n]), step_time, n)
                cur = full_pose(cfg["poses"][n])

        order = list(cfg["cycle"])
        if a.start:
            if a.start not in order:
                sys.exit(f"{a.start} is not in the cycle")
            i = order.index(a.start)
            order = order[i:] + order[:i]

        lap = 0
        while cycles == 0 or lap < cycles:
            lap += 1
            if cycles != 1:
                print(f"[cycle {lap}]")
            if a.watch:
                cfg = load(a.json)
            for n in order:
                r.blend(cur, full_pose(cfg["poses"][n]), step_time, n)
                cur = full_pose(cfg["poses"][n])
    except KeyboardInterrupt:
        print("\ninterrupted")
    finally:
        print("returning to neutral")
        r.blend(cur, full_pose({}), 0.8, "NEUTRAL")


if __name__ == "__main__":
    main()
