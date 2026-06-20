#pragma once
#include <stdint.h>
#include "../gait/stabilizer.h"   // for IMUEstimate

// ─── MPU6050 driver + complementary filter ────────────────────────────────────
// Reads raw gyro/accel via I2C and produces fused pitch/roll estimates.
//
// Complementary filter:
//   θ_est = α·(θ_prev + ω·Δt) + (1-α)·θ_accel
//   α = 0.98

class MPU6050 {
public:
    MPU6050();

    // Initialize I2C and configure sensor. Returns false if device not found.
    bool begin(uint8_t addr, int sda, int scl, uint32_t freq);

    // Read raw sensor data and apply complementary filter.
    // Call at TASK_IMU_HZ frequency. Returns false on I2C error.
    bool update(float dt_s);

    // Get latest fused estimates
    IMUEstimate estimate() const;

    // Raw values (for telemetry)
    struct RawData {
        int16_t accel_x, accel_y, accel_z;
        int16_t gyro_x,  gyro_y,  gyro_z;
        int16_t temp_raw;
    };
    RawData raw() const { return raw_; }

    // Calibrate: average N samples with robot stationary
    void calibrate(int n_samples = 200);

private:
    uint8_t addr_;
    float   pitch_;      // [rad] forward lean, rotation around X
    float   roll_;       // [rad] right lean, rotation around Z
    float   yaw_rate_;   // [rad/s] rotation around Y (gyro only, no fusion)
    float   alpha_;      // complementary filter coefficient
    RawData raw_;

    // Calibration offsets
    float gyro_x_bias_, gyro_y_bias_, gyro_z_bias_;

    bool read_raw();
    static constexpr float ACCEL_SCALE = 1.0f / 16384.0f;  // ±2g range
    static constexpr float GYRO_SCALE  = 1.0f / 131.0f;    // ±250°/s range, result in °/s
    static constexpr float DEG2RAD     = 0.01745329252f;
};

// MPU6050 register addresses
namespace MPU6050Reg {
    constexpr uint8_t PWR_MGMT_1   = 0x6B;
    constexpr uint8_t ACCEL_XOUT_H = 0x3B;
    constexpr uint8_t GYRO_XOUT_H  = 0x43;
    constexpr uint8_t TEMP_OUT_H   = 0x41;
    constexpr uint8_t SMPLRT_DIV   = 0x19;
    constexpr uint8_t CONFIG       = 0x1A;
    constexpr uint8_t ACCEL_CONFIG = 0x1C;
    constexpr uint8_t GYRO_CONFIG  = 0x1B;
    constexpr uint8_t WHO_AM_I     = 0x75;
}
