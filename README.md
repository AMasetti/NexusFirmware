# Optimus — Biped Robot

8-DOF bipedal robot with CPG gait, sagittal IK sway, and real-time IMU stabilization. ESP32-C3 + MPU6050 + PCA9685 + 8× MG995 leg servos + 6× Futaba S3003 arm servos.

Telemetry and control are exposed over WebSocket at `ws://optimus.local:81` (mDNS — no IP lookup needed). The `futurespace-ui` digital twin connects via a ROS 2 bridge for live monitoring and remote servo control.

## Hardware

| Component | Part | Interface |
|---|---|---|
| MCU | ESP32-C3 Super Mini | USB-CDC (single core, 160 MHz) |
| IMU | MPU6050 | I2C @ 0x68, 400 kHz |
| Servo driver | PCA9685 | I2C @ 0x40, 400 kHz |
| Leg servos ×8 | MG995 | PWM 600–2650 µs @ 50 Hz |
| Arm servos ×6 | Futaba S3003 | PWM 900–2100 µs @ 50 Hz |
| I2C pins | SDA=GPIO8, SCL=GPIO9 | — |
| Servo power | 6 V dedicated rail | — |

## Leg geometry

```
HIP ROLL (θ₁) → HIP PITCH (θ₂)
                      │
                 L1 = 90 mm  (femur)
                      │
                 KNEE (θ₃)
                      │
                 L2 = 90 mm  (tibia)
                      │
              ANKLE ROLL (θ₄) → L3 = 30 mm (foot)
```

**Ankle constraint:** `θ₄ = −θ₁` always — keeps the foot flat.

## PCA9685 channel map

| Ch | Joint | Direction | Halt° |
|---|---|---|---|
| 0 | R ankle_roll | +1 | 90 |
| 1 | R knee | +1 | 80 |
| 2 | R hip_pitch | −1 | 100 |
| 3 | R hip_roll | −1 | 100 |
| 4 | R shoulder FB | +1 | 90 |
| 5 | R shoulder lat | +1 | 90 |
| 6 | R forearm lat | +1 | 90 |
| 7 | L shoulder FB | +1 | 90 |
| 8 | L shoulder lat | +1 | 90 |
| 9 | L forearm lat | +1 | 90 |
| 10 | Hip yaw | +1 | 90 |
| 12 | L hip_roll | +1 | 80 |
| 13 | L hip_pitch | +1 | 90 |
| 14 | L knee | −1 | 90 |
| 15 | L ankle_roll | −1 | 70 |

## FreeRTOS tasks

| Task | Hz | Priority | Stack |
|---|---|---|---|
| `task_imu` | 200 | 3 | 4 KB |
| active experiment | 100 | 2 | 8 KB |
| `task_telemetry` | 50 (broadcasts at 10) | 1 | 8 KB |

All tasks pinned to core 0. Shared state (`g_imu_estimate`, `g_leg_angles`) protected by `state_mutex`.

## Active experiments (one enabled at a time)

Set exactly one flag to `1` in `include/config.h` — a `static_assert` enforces mutual exclusivity.

| Flag | Description |
|---|---|
| `ENABLE_IK_SWAY` ← **default** | Both feet fixed, hip oscillates sagittally via IK |
| `ENABLE_GAIT` | Open-loop CPG sinusoidal walk |
| `ENABLE_WALK` | IMU-gated 4-phase walk state machine |
| `ENABLE_BALANCE` | Static two-foot IMU balance |
| `ENABLE_SINGLE_LEG_BALANCE` | Balance on right foot only |

## Build & flash

Requires **Arduino CLI** and the ESP32 core.

```bash
cd optimus/firmware

make setup          # first time: installs ESP32 core + WebSockets library
make compile        # build only (run this after every edit)
make compile-upload # build + flash
make monitor        # serial monitor @ 115200 baud
```

**Always run `make compile` after editing `src/` or `include/`.** The compile-check skill enforces this automatically.

### First-time setup

```bash
arduino-cli config add board_manager.additional_urls \
  https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
arduino-cli core update-index
arduino-cli core install esp32:esp32
arduino-cli lib install "WebSockets"
```

### Flashing the ESP32-C3 Super Mini

The Super Mini has no auto-reset circuit. **Hold BOOT, plug in USB, release BOOT** before uploading.

## WiFi & network

Credentials live in `firmware/include/secrets.h` (git-ignored). Copy the template and fill in your values:

```bash
cp firmware/include/secrets.h.example firmware/include/secrets.h
```

```c
// secrets.h
#define WIFI_SSID      "your_network"
#define WIFI_PASSWORD  "your_password"
```

Other network settings in `firmware/include/config.h`:

```c
#define WS_PORT       81
#define MDNS_HOSTNAME "optimus"   // reachable as optimus.local
```

On boot the robot prints its MAC and announces `optimus.local` via mDNS. Reserve the MAC in your router for a static IP as a backup.

## Telemetry WebSocket

Connect to `ws://optimus.local:81`.

**Outgoing (10 Hz):**
```json
{
  "t": 12345,
  "imu": {
    "pitch": 0.02, "roll": -0.01, "yaw_rate": 0.001,
    "ax": 0.12, "ay": 9.78, "az": 0.05,
    "gx": 0.001, "gy": 0.0, "gz": -0.002
  },
  "joints": {
    "l_hip_roll": 0.0, "l_hip_pitch": 0.23, "l_knee": 0.18, "l_ankle_roll": 0.0,
    "r_hip_roll": 0.0, "r_hip_pitch": -0.23, "r_knee": 0.18, "r_ankle_roll": 0.0
  }
}
```

**Incoming commands:**
```json
{ "cmd": "set_joint", "joint": "l_knee", "value": 0.3 }
{ "cmd": "set_neutral" }
{ "cmd": "calibrate_imu" }
{ "cmd": "set_period",  "value": 1.0 }
{ "cmd": "set_amp_hp",  "value": 0.35 }
{ "cmd": "set_amp_hr",  "value": 0.15 }
{ "cmd": "set_amp_k",   "value": 0.25 }
{ "cmd": "set_amp_ar",  "value": 0.10 }
```

## Digital twin (futurespace-ui)

The `../futurespace-ui/` Next.js app connects via a ROS 2 bridge running in Docker:

```bash
cd ../docker
docker compose up --build
# UI at http://localhost:3010/robotics
```

The bridge connects to `ws://optimus.local:81`, converts telemetry to ROS 2 topics, and exposes them to the browser via rosbridge (`wss://localhost:9090`).

## File structure

```
optimus/
├── firmware/
│   ├── firmware.ino          # Arduino CLI entry point
│   ├── sketch.yaml           # board FQBN + default port
│   ├── Makefile              # compile / upload / monitor targets
│   ├── include/
│   │   └── config.h          # all tunable constants and mode flags
│   └── src/
│       ├── main.cpp          # FreeRTOS tasks, shared globals, write_leg()
│       ├── gait/
│       │   ├── cpg.cpp/h     # CPG sinusoidal oscillator
│       │   ├── stabilizer.cpp/h  # IMU feedback corrections
│       │   └── walk.cpp/h    # 4-phase IMU-gated walk
│       ├── kinematics/
│       │   ├── forward.cpp/h # FK (analytical + DH cross-check)
│       │   └── inverse.cpp/h # IK (5-step decoupled)
│       ├── imu/
│       │   └── mpu6050.cpp/h # Driver + complementary filter (α=0.98)
│       ├── servo/
│       │   └── pca9685.cpp/h # PWM driver
│       ├── comms/
│       │   └── telemetry.cpp/h  # WiFi + WebSocket server + mDNS
│       └── experiments/
│           ├── exp_ik_sway.cpp/h   # IK sagittal sway (current default)
│           ├── exp_gait.cpp/h      # open-loop CPG
│           ├── exp_walk.cpp/h      # IMU-gated walk
│           ├── exp_balance.cpp/h   # two-foot balance
│           └── exp_slb.cpp/h       # single-leg balance
├── docs/
│   ├── JOINTS.md             # hardware reference, channel map, servo direction table
│   ├── kinematics.md         # FK/IK derivations, DH parameters, workspace analysis
│   └── imu-balance.md        # IMU axis orientation, complementary filter, balance gains
└── tools/
    └── joycon_sway.py        # DEPRECATED — replaced by futurespace-ui override mode
```

## Safety

The IMU safety cutoff halts all servos and suspends the active task if tilt exceeds `SAFETY_TILT_LIMIT_DEG = 50°`. A board reset is required to resume. Do not disable this when the robot is powered on legs.
