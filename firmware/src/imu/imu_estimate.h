#pragma once

// Fused IMU attitude shared by the driver, stabilizer and telemetry.
struct IMUEstimate {
    float pitch_rad;          // positive = tilting forward  (rotation around X)
    float roll_rad;           // positive = tilting right    (rotation around Z)
    float yaw_rate_rad_s;     // positive = rotating CCW from above (rotation around Y)
};
