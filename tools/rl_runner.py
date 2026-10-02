"""
rl_runner.py — WiFi inference runner for Optimus RL policy (Option A).

Connects to the ESP32 running firmware-rl via WebSocket, receives IMU +
joint state, runs CPG + PPO residual at 50 Hz, sends absolute joint angles back.

Usage:
    python rl_runner.py --model <path/to/best_model.zip> [--host optimus-rl.local]

The model directory must contain vecnorm.pkl alongside best_model.zip.

Architecture:
    PC (this script, 50 Hz)
      ├─ CPG oscillator (mirrors mujuco/optimus/rl/cpg.py)
      ├─ VecNormalize + PPO policy (Run 27 best_model.zip)
      ├─ receives: IMU pitch/roll/yaw_rate + 15 joint angles from ESP32
      └─ sends: 15 absolute joint angles to ESP32

Joint name mapping:
    sim (optimus_cpg_env ACTUATOR_NAMES) → firmware-rl WebSocket field
    The order must match ACTUATOR_NAMES exactly for obs construction.
"""

import argparse, os, sys, time, json, threading, queue
import numpy as np
import websocket  # pip install websocket-client

# ── Path setup ────────────────────────────────────────────────────────────────
_REPO = os.path.abspath(os.path.join(os.path.dirname(__file__), "../.."))
sys.path.insert(0, os.path.join(_REPO, "mujuco/optimus/rl"))

from cpg import CPG  # reuse sim CPG directly

# ── Joint name mapping ────────────────────────────────────────────────────────
# Must match ACTUATOR_NAMES in optimus_cpg_env.py exactly (same index order).
ACTUATOR_NAMES = [
    "Servo-Hip-L",
    "Servo-Knee-L-Top",
    "Servo-Knee-L-Bottom",
    "Servo-Ankle-L",
    "Servo-Hip-R",
    "Servo-Knee-R-Top",
    "Servo-Knee-R-Bottom",
    "Servo-Ankle-R",
    "Servo-Hip-Body-Rotation",
    "Servo-Showlder-L-Front-Back",
    "Servo-Showlder-R-Front-Back",
    "Servo-Showlder-L-Inward-Outward",
    "Servo-Showlder-R-Inward-Outward",
    "Servo-Forearm-L",
    "Servo-Forearm-R",
]

# sim actuator name → firmware-rl WebSocket joint field
SIM_TO_HW = {
    "Servo-Hip-L":                     "l_hip_roll",
    "Servo-Knee-L-Top":                "l_hip_pitch",   # knee top = hip pitch in firmware
    "Servo-Knee-L-Bottom":             "l_knee",
    "Servo-Ankle-L":                   "l_ankle_roll",
    "Servo-Hip-R":                     "r_hip_roll",
    "Servo-Knee-R-Top":                "r_hip_pitch",
    "Servo-Knee-R-Bottom":             "r_knee",
    "Servo-Ankle-R":                   "r_ankle_roll",
    "Servo-Hip-Body-Rotation":         "hip_yaw",
    "Servo-Showlder-L-Front-Back":     "l_shoulder_fb",
    "Servo-Showlder-R-Front-Back":     "r_shoulder_fb",
    "Servo-Showlder-L-Inward-Outward": "l_shoulder_lat",
    "Servo-Showlder-R-Inward-Outward": "r_shoulder_lat",
    "Servo-Forearm-L":                 "l_forearm_lat",
    "Servo-Forearm-R":                 "r_forearm_lat",
}

# firmware-rl joint field → index into the 15-dim joint obs vector
HW_JOINT_NAMES = [
    "l_hip_roll", "l_hip_pitch", "l_knee", "l_ankle_roll",
    "r_hip_roll", "r_hip_pitch", "r_knee", "r_ankle_roll",
    "l_shoulder_fb", "r_shoulder_fb", "l_shoulder_lat", "r_shoulder_lat",
    "l_forearm_lat", "r_forearm_lat", "hip_yaw",
]
HW_TO_IDX = {n: i for i, n in enumerate(HW_JOINT_NAMES)}

# Full 23-joint order from qpos[7:] — must match XML joint order exactly.
# Unactuated (passive/parallelogram) joints are reconstructed via constraints:
#   Unactuated-Knee-X-Top    = -Servo-Knee-X-Top
#   Unactuated-Tendon-X-Top  = -Servo-Knee-X-Top
#   Unactuated-Knee-X-Bottom  = -Servo-Knee-X-Bottom
#   Unactuated-Tendon-X-Bottom = +Servo-Knee-X-Bottom
# Each entry: (hw_name_or_None, sign_if_unactuated, source_hw_name_if_unactuated)
_JPOS_MAP = [
    ("hip_yaw",       1,  None),               # 0  Servo-Hip-Body-Rotation
    ("l_shoulder_fb", 1,  None),               # 1  Servo-Showlder-L-Front-Back
    ("l_shoulder_lat",1,  None),               # 2  Servo-Showlder-L-Inward-Outward
    ("l_forearm_lat", 1,  None),               # 3  Servo-Forearm-L
    ("r_shoulder_fb", 1,  None),               # 4  Servo-Showlder-R-Front-Back
    ("r_shoulder_lat",1,  None),               # 5  Servo-Showlder-R-Inward-Outward
    ("r_forearm_lat", 1,  None),               # 6  Servo-Forearm-R
    ("l_hip_roll",    1,  None),               # 7  Servo-Hip-L
    (None,           -1,  "l_hip_pitch"),      # 8  Unactuated-Knee-L-Top
    ("l_hip_pitch",   1,  None),               # 9  Servo-Knee-L-Top
    ("l_knee",        1,  None),               # 10 Servo-Knee-L-Bottom
    (None,           -1,  "l_knee"),           # 11 Unactuated-Knee-L-Bottom
    (None,           +1,  "l_knee"),           # 12 Unactuated-Tendon-L-Bottom
    ("l_ankle_roll",  1,  None),               # 13 Servo-Ankle-L
    (None,           -1,  "l_hip_pitch"),      # 14 Unactuated-Tendon-L-Top
    ("r_hip_roll",    1,  None),               # 15 Servo-Hip-R
    (None,           -1,  "r_hip_pitch"),      # 16 Unactuated-Knee-R-Top
    ("r_hip_pitch",   1,  None),               # 17 Servo-Knee-R-Top
    ("r_knee",        1,  None),               # 18 Servo-Knee-R-Bottom
    (None,           -1,  "r_knee"),           # 19 Unactuated-Knee-R-Bottom
    (None,           +1,  "r_knee"),           # 20 Unactuated-Tendon-R-Bottom
    ("r_ankle_roll",  1,  None),               # 21 Servo-Ankle-R
    (None,           -1,  "r_hip_pitch"),      # 22 Unactuated-Tendon-R-Top
]

MAX_RESIDUAL = 0.20   # rad — must match optimus_cpg_env.MAX_RESIDUAL

_SIM_AXIS_FLIP: set = set()

# Arm command smoothing — see the filter in run(). Futaba S3003 servos cannot
# follow the policy's raw per-step jumps and buzz. alpha=0.25 at 50 Hz gives a
# ~25 ms time constant: fast enough to keep the arm swing, slow enough to stop
# the hunting. Raise toward 1.0 for more responsiveness, lower for less buzz.
ARM_SMOOTH_ALPHA = 0.25

# Swap the left/right ankle commands before sending to the ESP32. Observed on
# hardware: the swing-foot lift appeared on the stance ankle. Set False to
# disable if the ankle wiring/mapping is corrected at the source.
SWAP_HW_ANKLES = False
_ARM_SMOOTH_JOINTS = (
    "l_shoulder_fb", "r_shoulder_fb",
    "l_shoulder_lat", "r_shoulder_lat",
    "l_forearm_lat", "r_forearm_lat",
    "hip_yaw",
)

# Walking pose offset: firmware halt = T-pose (0 rad), RL walking = arms hanging.
# L needs -90°, R needs +90° (servo mounted mirrored on R side).
_HW_OFFSETS = {
    "l_shoulder_lat": np.radians(-90.0),
    "r_shoulder_lat": np.radians(+90.0),
}


class RLRunner:
    def __init__(self, model_path: str, host: str, port: int = 81,
                 ctrl_dt: float = 0.02, verbose: bool = False):
        self.ctrl_dt = ctrl_dt
        self.verbose = verbose
        self._host   = host
        self._port   = port

        # Load model + vecnorm
        from stable_baselines3 import PPO
        from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

        model_path = os.path.abspath(model_path)
        run_dir    = os.path.dirname(model_path)
        vn_path    = os.path.join(run_dir, "vecnorm.pkl")

        if os.path.exists(vn_path):
            # Load stats directly — no dummy env needed for inference-only normalisation
            import pickle
            with open(vn_path, "rb") as f:
                vn_data = pickle.load(f)
            # vn_data is a VecNormalize instance; extract running mean/var
            self._obs_rms = vn_data.obs_rms
            self._clip_obs = vn_data.clip_obs
            print(f"[runner] VecNormalize loaded from {vn_path}")
        else:
            self._obs_rms = None
            self._clip_obs = 10.0
            print("[runner] WARNING: no vecnorm.pkl — obs will be unnormalised")

        # SB3 appends .zip if not already stripped — strip it to avoid .zip.zip
        model_path_load = model_path[:-4] if model_path.endswith(".zip") else model_path
        self._model = PPO.load(model_path_load, device="cpu")
        print(f"[runner] Model loaded: {model_path}")

        self._cpg = CPG()

        # Latest state from ESP32 (15 actuated joints, hw-name indexed)
        self._imu_pitch    = 0.0
        self._imu_roll     = 0.0
        self._imu_yaw_rate = 0.0
        self._hw_pos  = {n: 0.0 for n in HW_JOINT_NAMES}
        self._hw_vel  = {n: 0.0 for n in HW_JOINT_NAMES}
        self._prev_hw = {n: 0.0 for n in HW_JOINT_NAMES}
        # Last filtered arm command (see ARM_SMOOTH_ALPHA); empty = not yet seeded.
        self._arm_filt: dict = {}
        self._state_lock   = threading.Lock()
        self._state_q      = queue.Queue(maxsize=4)

        self._ws           = None
        self._running      = False
        self._connected_ev = threading.Event()

    # ── WebSocket callbacks ───────────────────────────────────────────────────

    def _on_message(self, ws, message):
        try:
            d = json.loads(message)
        except Exception:
            return

        imu    = d.get("imu", {})
        joints = d.get("joints", {})

        with self._state_lock:
            self._imu_pitch    = float(imu.get("pitch",    0.0))
            self._imu_roll     = float(imu.get("roll",     0.0))
            self._imu_yaw_rate = float(imu.get("yaw_rate", 0.0))

            for n in HW_JOINT_NAMES:
                new_v = float(joints.get(n, 0.0))
                self._hw_vel[n] = (new_v - self._prev_hw[n]) / self.ctrl_dt
                self._prev_hw[n] = new_v
                self._hw_pos[n]  = new_v

    def _on_error(self, ws, error):
        print(f"[WS] Error: {error}")

    def _on_close(self, ws, *_):
        print("[WS] Connection closed")
        self._running = False

    def _on_open(self, ws):
        print("[WS] Connected to ESP32")
        self._running = True
        self._connected_ev.set()

    # ── Obs construction (mirrors optimus_cpg_env._get_obs) ──────────────────

    def _build_jvec(self, use_vel: bool) -> np.ndarray:
        src = self._hw_vel if use_vel else self._hw_pos
        out = np.zeros(23, dtype=np.float32)
        for idx, (hw_name, sign, src_name) in enumerate(_JPOS_MAP):
            if hw_name is not None:
                out[idx] = float(src[hw_name])
            else:
                out[idx] = sign * float(src[src_name])
        return out

    def _get_obs(self) -> np.ndarray:
        with self._state_lock:
            cpg_phase = np.array([
                np.sin(self._cpg._omega * self._cpg._t),
                np.cos(self._cpg._omega * self._cpg._t),
            ], dtype=np.float32)
            imu = np.array([self._imu_pitch, self._imu_roll, self._imu_yaw_rate],
                           dtype=np.float32)
            jpos = self._build_jvec(use_vel=False)
            jvel = self._build_jvec(use_vel=True)

        return np.concatenate([cpg_phase, imu, jpos, jvel])

    def _normalize_obs(self, obs: np.ndarray) -> np.ndarray:
        if self._obs_rms is None:
            return obs
        obs_norm = (obs - self._obs_rms.mean) / np.sqrt(self._obs_rms.var + 1e-8)
        return np.clip(obs_norm, -self._clip_obs, self._clip_obs).astype(np.float32)

    # ── Main control loop ─────────────────────────────────────────────────────

    def _send_joints(self, targets: dict):
        angles = {hw: float(targets.get(hw, 0.0)) for hw in HW_JOINT_NAMES}
        msg = json.dumps({"cmd": "set_joints", "angles": angles})
        try:
            self._ws.send(msg)
        except Exception as e:
            print(f"[WS] Send error: {e}")

    def run(self):
        url = f"ws://{self._host}:{self._port}"
        print(f"[runner] Connecting to {url}...")

        self._ws = websocket.WebSocketApp(
            url,
            on_message=self._on_message,
            on_error=self._on_error,
            on_close=self._on_close,
            on_open=self._on_open,
        )
        # WebSocket recv runs in background thread
        ws_thread = threading.Thread(target=self._ws.run_forever, daemon=True)
        ws_thread.start()

        # Wait for connection (up to 15s to allow mDNS resolution)
        if not self._connected_ev.wait(timeout=15.0):
            print("[runner] Could not connect — is firmware-rl running?")
            return

        # Wait for first telemetry packet before starting control loop
        print("[runner] Waiting for first telemetry...")
        for _ in range(100):
            with self._state_lock:
                got_data = any(v != 0.0 for v in self._hw_pos.values())
            if got_data:
                break
            time.sleep(0.05)

        print("[runner] Control loop running at 50 Hz. Ctrl+C to stop.")
        step = 0
        try:
            while self._running:
                t0 = time.perf_counter()

                obs_raw  = self._get_obs()
                obs_norm = self._normalize_obs(obs_raw)

                action, _ = self._model.predict(obs_norm[np.newaxis, :], deterministic=True)
                action = action[0]  # (15,)

                # CPG step + IMU stabilisation (mirrors sim env)
                with self._state_lock:
                    pitch = self._imu_pitch
                    roll  = self._imu_roll
                    yaw_r = self._imu_yaw_rate
                cpg_targets = self._cpg.step(self.ctrl_dt, roll=roll)
                cpg_targets = self._cpg.stabilise(cpg_targets, pitch, roll, yaw_r)

                # Add RL residual — action is in [-1,1], scale by MAX_RESIDUAL (matches env)
                hw_angles = {}
                for i, sim_name in enumerate(ACTUATOR_NAMES):
                    hw_name = SIM_TO_HW[sim_name]
                    base    = cpg_targets.get(sim_name, 0.0)
                    val     = float(base + MAX_RESIDUAL * action[i])
                    if hw_name in _HW_OFFSETS:
                        val += _HW_OFFSETS[hw_name]
                    if hw_name in _SIM_AXIS_FLIP:
                        val = -val
                    hw_angles[hw_name] = val

                # Hardware-only ankle swap. On the real robot the swing-foot lift
                # landed on the STANCE ankle: the ankle channels are mirrored
                # relative to the sim's L/R convention. Swapping here rather than
                # in cpg.py keeps the simulation correct (swapping it there makes
                # the sim robot fall at step 63 instead of walking 500).
                if SWAP_HW_ANKLES:
                    hw_angles["l_ankle_roll"], hw_angles["r_ankle_roll"] = \
                        hw_angles["r_ankle_roll"], hw_angles["l_ankle_roll"]

                # Smooth the arm commands. The policy's raw output jumps 6-11 deg
                # per 50 Hz step (300-540 deg/s); a Futaba S3003 is a slow analog
                # servo (~0.23 s/60 deg) and cannot track that, so it hunts and
                # buzzes. MuJoCo's PD + link inertia filter this in sim; the
                # runner sends it straight to the servo. Legs (MG995, higher
                # torque and speed) are left unfiltered so the gait is unchanged.
                for hw_name in _ARM_SMOOTH_JOINTS:
                    prev = self._arm_filt.get(hw_name)
                    cur  = hw_angles[hw_name]
                    hw_angles[hw_name] = cur if prev is None else \
                        prev + ARM_SMOOTH_ALPHA * (cur - prev)
                    self._arm_filt[hw_name] = hw_angles[hw_name]

                self._send_joints(hw_angles)

                if self.verbose and step % 50 == 0:
                    with self._state_lock:
                        p, r = self._imu_pitch, self._imu_roll
                    print(f"[step {step:5d}]  pitch={np.degrees(p):.1f}°  "
                          f"roll={np.degrees(r):.1f}°  "
                          f"|action|={np.abs(action).mean():.3f}")

                step += 1
                elapsed = time.perf_counter() - t0
                time.sleep(max(0.0, self.ctrl_dt - elapsed))

        except KeyboardInterrupt:
            print("\n[runner] Stopping — sending halt")
        finally:
            try:
                self._ws.send(json.dumps({"cmd": "halt"}))
                time.sleep(0.1)
            except Exception:
                pass
            self._ws.close()


def main():
    parser = argparse.ArgumentParser(description="Optimus RL WiFi runner")
    parser.add_argument("--model", required=True,
                        help="Path to best_model.zip (vecnorm.pkl must be alongside)")
    parser.add_argument("--host",  default="optimus-rl.local",
                        help="ESP32 hostname or IP (default: optimus-rl.local)")
    parser.add_argument("--port",  type=int, default=81)
    parser.add_argument("--verbose", action="store_true",
                        help="Print step diagnostics every second")
    args = parser.parse_args()

    runner = RLRunner(
        model_path=args.model,
        host=args.host,
        port=args.port,
        verbose=args.verbose,
    )
    runner.run()


if __name__ == "__main__":
    main()
