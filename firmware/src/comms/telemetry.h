#pragma once
#include "../kinematics/forward.h"
#include "../gait/stabilizer.h"
#include "../gait/cpg.h"
#include "../imu/mpu6050.h"

// ─── WebSocket Telemetry ───────────────────────────────────────────────────────
// Streams robot state at 10 Hz and receives parameter tuning commands.
//
// Outgoing JSON (every 100 ms):
// {
//   "t": <timestamp_ms>,
//   "imu": {
//     "pitch": <rad>, "roll": <rad>, "yaw_rate": <rad/s>,
//     "ax": <m/s²>, "ay": <m/s²>, "az": <m/s²>,
//     "gx": <rad/s>, "gy": <rad/s>, "gz": <rad/s>
//   },
//   "joints": {
//     "l_hip_roll": <rad>, "l_hip_pitch": <rad>, "l_knee": <rad>, "l_ankle_roll": <rad>,
//     "r_hip_roll": <rad>, "r_hip_pitch": <rad>, "r_knee": <rad>, "r_ankle_roll": <rad>,
//     "r_shoulder_fb": <rad>, "r_shoulder_lat": <rad>, "r_forearm_lat": <rad>,
//     "l_shoulder_fb": <rad>, "l_shoulder_lat": <rad>, "l_forearm_lat": <rad>,
//     "hip_yaw": <rad>
//   }
// }
//
// Incoming commands (JSON):
// { "cmd": "set_neutral"                               }
// { "cmd": "calibrate_imu"                             }
// { "cmd": "set_joint", "joint": "l_knee", "value": 0.3 }
// Legacy CPG commands (still handled for backward compat):
// { "cmd": "set_period",  "value": 1.0 }
// { "cmd": "set_amp_hp",  "value": 0.35 }
// { "cmd": "set_amp_hr",  "value": 0.15 }
// { "cmd": "set_amp_k",   "value": 0.25 }
// { "cmd": "set_amp_ar",  "value": 0.10 }
// { "cmd": "set_sway_target", "value": 10.0 }

struct RobotState {
    IMUEstimate      imu;       // pitch, roll, yaw_rate from complementary filter
    MPU6050::RawData raw;       // raw accel/gyro int16 values (scaled in build_json)
    LegAngles        legs;
    float            arms[7];   // r_shoulder_fb, r_shoulder_lat, r_forearm_lat, l_shoulder_fb, l_shoulder_lat, l_forearm_lat, hip_yaw
    CPGParams        cpg;       // retained for legacy command handling
    uint32_t         timestamp_ms;
};

class Telemetry {
public:
    Telemetry();

    // Start WiFi + WebSocket server. Call once from setup().
    bool begin(const char* ssid, const char* password, uint16_t port);

    // Loop call — process incoming packets (call from telemetry task)
    void loop();

    // Push current state to all connected clients
    void send_state(const RobotState& state);

    // Callback for commands received from client.
    // joint is set for "set_joint" commands, empty string otherwise.
    using ParamCallback = void (*)(const char* cmd, const char* joint, float value);
    void set_param_callback(ParamCallback cb);

    bool is_connected() const;

private:
    uint16_t      port_;
    ParamCallback param_cb_;
    bool          connected_;

    char tx_buf_[800];  // enlarged for full 15-DOF joint telemetry
    void build_json(const RobotState& state, char* buf, size_t buf_size);
};
