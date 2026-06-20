#include "mpu6050.h"
#include "../../include/config.h"
#include <Wire.h>
#include <math.h>
#include <Arduino.h>

MPU6050::MPU6050()
    : addr_(MPU6050_ADDR), pitch_(0.0f), roll_(0.0f), yaw_rate_(0.0f),
      alpha_(COMPLEMENTARY_ALPHA),
      gyro_x_bias_(0.0f), gyro_y_bias_(0.0f), gyro_z_bias_(0.0f) {
    memset(&raw_, 0, sizeof(raw_));
}

bool MPU6050::begin(uint8_t addr, int sda, int scl, uint32_t freq) {
    addr_ = addr;
    Wire.begin(sda, scl, freq);

    // Check WHO_AM_I register
    Wire.beginTransmission(addr_);
    Wire.write(MPU6050Reg::WHO_AM_I);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)addr_, (uint8_t)1);
    if (!Wire.available()) return false;
    uint8_t who = Wire.read();
    if (who != 0x68 && who != 0x70) return false;   // 0x70 = MPU6501

    // Wake up (clear sleep bit)
    Wire.beginTransmission(addr_);
    Wire.write(MPU6050Reg::PWR_MGMT_1);
    Wire.write(0x00);
    Wire.endTransmission();
    delay(100);

    // Sample rate divider: 1kHz / (1+1) = 500 Hz (more than enough)
    Wire.beginTransmission(addr_);
    Wire.write(MPU6050Reg::SMPLRT_DIV);
    Wire.write(0x01);
    Wire.endTransmission();

    // Digital low-pass filter: bandwidth ~44Hz
    Wire.beginTransmission(addr_);
    Wire.write(MPU6050Reg::CONFIG);
    Wire.write(0x03);
    Wire.endTransmission();

    // Gyro: ±250°/s
    Wire.beginTransmission(addr_);
    Wire.write(MPU6050Reg::GYRO_CONFIG);
    Wire.write(0x00);
    Wire.endTransmission();

    // Accel: ±2g
    Wire.beginTransmission(addr_);
    Wire.write(MPU6050Reg::ACCEL_CONFIG);
    Wire.write(0x00);
    Wire.endTransmission();

    return true;
}

bool MPU6050::read_raw() {
    Wire.beginTransmission(addr_);
    Wire.write(MPU6050Reg::ACCEL_XOUT_H);
    if (Wire.endTransmission(false) != 0) return false;

    Wire.requestFrom((uint8_t)addr_, (uint8_t)14);
    if (Wire.available() < 14) return false;

    raw_.accel_x = (int16_t)((Wire.read() << 8) | Wire.read());
    raw_.accel_y = (int16_t)((Wire.read() << 8) | Wire.read());
    raw_.accel_z = (int16_t)((Wire.read() << 8) | Wire.read());
    raw_.temp_raw = (int16_t)((Wire.read() << 8) | Wire.read());
    raw_.gyro_x  = (int16_t)((Wire.read() << 8) | Wire.read());
    raw_.gyro_y  = (int16_t)((Wire.read() << 8) | Wire.read());
    raw_.gyro_z  = (int16_t)((Wire.read() << 8) | Wire.read());
    return true;
}

bool MPU6050::update(float dt_s) {
    if (!read_raw()) return false;

    // Convert to physical units
    float ax = raw_.accel_x * ACCEL_SCALE;   // [g]
    float ay = raw_.accel_y * ACCEL_SCALE;   // [g]  ≈ +1 when upright (Y = up)
    float az = raw_.accel_z * ACCEL_SCALE;   // [g]

    // Physical orientation: X = right→left, Y = feet→head (up), Z = rear→front
    //
    // body_pitch (forward lean, rotation around X):
    //   pitch_accel = atan2(-az, ay)  — az gains gravity component as head dips forward
    //   gyro term   = gyro_x
    //
    // body_roll (lateral lean, rotation around Z):
    //   roll_accel  = atan2( ax, ay)  — ax gains gravity component as robot leans right
    //   gyro term   = gyro_z
    //
    // yaw_rate (twist around vertical Y): gyro_y only (no accel fusion for absolute yaw)

    float gx = (raw_.gyro_x * GYRO_SCALE - gyro_x_bias_) * DEG2RAD;   // [rad/s] around X
    float gz = (raw_.gyro_z * GYRO_SCALE - gyro_z_bias_) * DEG2RAD;   // [rad/s] around Z
    float gy = (raw_.gyro_y * GYRO_SCALE - gyro_y_bias_) * DEG2RAD;   // [rad/s] around Y

    float pitch_accel = IMU_PITCH_SIGN * atan2f(-az, ay);
    float roll_accel  = IMU_ROLL_SIGN  * atan2f( ax, ay);

    // Complementary filter
    pitch_ = alpha_ * (pitch_ + IMU_PITCH_SIGN * gx * dt_s) + (1.0f - alpha_) * pitch_accel;
    roll_  = alpha_ * (roll_  + IMU_ROLL_SIGN  * gz * dt_s) + (1.0f - alpha_) * roll_accel;
    yaw_rate_ = gy;

    return true;
}

IMUEstimate MPU6050::estimate() const {
    return { pitch_, roll_, yaw_rate_ };
}

void MPU6050::calibrate(int n_samples) {
    float gx_sum = 0, gy_sum = 0, gz_sum = 0;
    for (int i = 0; i < n_samples; i++) {
        read_raw();
        gx_sum += raw_.gyro_x * GYRO_SCALE;
        gy_sum += raw_.gyro_y * GYRO_SCALE;
        gz_sum += raw_.gyro_z * GYRO_SCALE;
        delay(5);
    }
    gyro_x_bias_ = gx_sum / n_samples;
    gyro_y_bias_ = gy_sum / n_samples;
    gyro_z_bias_ = gz_sum / n_samples;
}
