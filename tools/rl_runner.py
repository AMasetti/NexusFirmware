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

# sim obs joint order (matches optimus_cpg_env _get_obs jpos ordering)
# This must match the XML joint order — same as ACTUATOR_NAMES mapped to hw names.
SIM_OBS_ORDER = [SIM_TO_HW[a] for a in ACTUATOR_NAMES]


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

        run_dir  = os.path.dirname(os.path.abspath(model_path))
        vn_path  = os.path.join(run_dir, "vecnorm.pkl")

        # Dummy env just to satisfy VecNormalize API — never stepped
        import gymnasium as gym
        dummy = DummyVecEnv([lambda: gym.make("Pendulum-v1")])
        if os.path.exists(vn_path):
            self._vnorm = VecNormalize.load(vn_path, dummy)
            self._vnorm.training  = False
            self._vnorm.norm_reward = False
            print(f"[runner] VecNormalize loaded from {vn_path}")
        else:
            self._vnorm = None
            print("[runner] WARNING: no vecnorm.pkl — obs will be unnormalised")

        self._model = PPO.load(model_path, device="cpu")
        print(f"[runner] Model loaded: {model_path}")

        self._cpg = CPG()

        # Latest state from ESP32
        self._imu_pitch    = 0.0
        self._imu_roll     = 0.0
        self._imu_yaw_rate = 0.0
        self._joint_pos    = np.zeros(15, dtype=np.float32)
        self._joint_vel    = np.zeros(15, dtype=np.float32)  # estimated via finite diff
        self._prev_joint   = np.zeros(15, dtype=np.float32)
        self._state_lock   = threading.Lock()
        self._state_q      = queue.Queue(maxsize=4)

        self._ws      = None
        self._running = False

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

            new_pos = np.array(
                [float(joints.get(n, 0.0)) for n in SIM_OBS_ORDER],
                dtype=np.float32
            )
            self._joint_vel  = (new_pos - self._prev_joint) / self.ctrl_dt
            self._prev_joint = new_pos.copy()
            self._joint_pos  = new_pos

    def _on_error(self, ws, error):
        print(f"[WS] Error: {error}")

    def _on_close(self, ws, *_):
        print("[WS] Connection closed")
        self._running = False

    def _on_open(self, ws):
        print("[WS] Connected to ESP32")
        self._running = True

    # ── Obs construction (mirrors optimus_cpg_env._get_obs) ──────────────────

    def _get_obs(self) -> np.ndarray:
        with self._state_lock:
            cpg_phase = np.array([
                np.sin(self._cpg._omega * self._cpg._t),
                np.cos(self._cpg._omega * self._cpg._t),
            ], dtype=np.float32)
            imu = np.array([self._imu_pitch, self._imu_roll, self._imu_yaw_rate],
                           dtype=np.float32)
            jpos = self._joint_pos.copy()
            jvel = self._joint_vel.copy()

        return np.concatenate([cpg_phase, imu, jpos, jvel])

    def _normalize_obs(self, obs: np.ndarray) -> np.ndarray:
        if self._vnorm is None:
            return obs
        # VecNormalize expects shape (n_envs, obs_dim)
        obs_2d = obs[np.newaxis, :]
        return self._vnorm.normalize_obs(obs_2d)[0]

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

        # Wait for connection
        for _ in range(50):
            if self._running:
                break
            time.sleep(0.1)
        if not self._running:
            print("[runner] Could not connect — is firmware-rl running?")
            return

        print("[runner] Control loop running at 50 Hz. Ctrl+C to stop.")
        step = 0
        try:
            while self._running:
                t0 = time.perf_counter()

                obs_raw  = self._get_obs()
                obs_norm = self._normalize_obs(obs_raw)

                action, _ = self._model.predict(obs_norm[np.newaxis, :], deterministic=True)
                action = action[0]  # (15,)

                # CPG step — pass current IMU roll for phase reset
                with self._state_lock:
                    roll = self._imu_roll
                cpg_targets = self._cpg.step(self.ctrl_dt, roll=roll)

                # Add RL residual (±0.20 rad, matches MAX_RESIDUAL in env)
                hw_angles = {}
                for i, sim_name in enumerate(ACTUATOR_NAMES):
                    hw_name = SIM_TO_HW[sim_name]
                    base    = cpg_targets.get(sim_name, 0.0)
                    hw_angles[hw_name] = float(base + action[i])

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
