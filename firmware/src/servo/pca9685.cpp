#include "pca9685.h"
#include "../../include/config.h"
#include <Wire.h>
#include <Arduino.h>
#include <math.h>

PCA9685::PCA9685() : addr_(PCA9685_ADDR), osc_freq_(INTERNAL_OSC_HZ), us_per_tick_(0.0f) {}

void PCA9685::write_reg(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(addr_);
    Wire.write(reg);
    Wire.write(value);
    Wire.endTransmission();
}

uint8_t PCA9685::read_reg(uint8_t reg) {
    Wire.beginTransmission(addr_);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom((uint8_t)addr_, (uint8_t)1);
    return Wire.available() ? Wire.read() : 0;
}

bool PCA9685::begin(uint8_t addr, uint32_t pwm_freq_hz) {
    addr_ = addr;

    // Reset
    write_reg(PCA9685Reg::MODE1, 0x00);
    delay(10);

    // Check presence: attempt a transmission and verify the device ACKs.
    Wire.beginTransmission(addr_);
    if (Wire.endTransmission() != 0) return false;   // no ACK → no device

    // Set PWM frequency
    // prescale = round(osc_freq / (4096 * freq)) - 1
    uint8_t prescale = (uint8_t)roundf((float)osc_freq_ / (PWM_RESOLUTION * pwm_freq_hz)) - 1;

    // Must be in sleep mode to set prescaler
    write_reg(PCA9685Reg::MODE1, 0x10);  // SLEEP=1, AI=0 (safe state for prescaler write)
    delay(5);
    write_reg(PCA9685Reg::PRESCALE, prescale);

    // Wake up with AI (auto-increment) bit set.
    // AI=1 (bit5) is required for multi-byte channel writes to advance registers.
    // Without it every byte in set_pwm() overwrites the same register → OFF stays 0 → no pulse.
    write_reg(PCA9685Reg::MODE1, 0x20);  // AI=1, SLEEP=0
    delay(5);
    // RESTART clears internal counters so all channels begin outputting immediately
    write_reg(PCA9685Reg::MODE1, 0xA0);  // RESTART=1, AI=1
    delay(5);

    // Compute us/tick: period_us / 4096
    float period_us = 1000000.0f / pwm_freq_hz;
    us_per_tick_ = period_us / PWM_RESOLUTION;

    // MODE2: totem-pole outputs (required to drive servo signal lines directly)
    write_reg(PCA9685Reg::MODE2, 0x04);

    // ── Diagnostic readback ──────────────────────────────────────────────────
    uint8_t mode1_rb    = read_reg(PCA9685Reg::MODE1);
    uint8_t mode2_rb    = read_reg(PCA9685Reg::MODE2);
    uint8_t prescale_rb = read_reg(PCA9685Reg::PRESCALE);
    Serial.printf("[PCA9685] MODE1=0x%02X  MODE2=0x%02X  PRESCALE=%u (want %u)\n",
                  mode1_rb, mode2_rb, prescale_rb, prescale);
    Serial.printf("[PCA9685] us_per_tick=%.4f  => actual freq ~%.1f Hz\n",
                  us_per_tick_,
                  1000000.0f / (us_per_tick_ * PWM_RESOLUTION));

    return true;
}

void PCA9685::set_pwm(uint8_t channel, uint16_t on_tick, uint16_t off_tick) {
    uint8_t base = PCA9685Reg::LED0_ON_L + 4 * channel;
    Wire.beginTransmission(addr_);
    Wire.write(base);
    Wire.write(on_tick  & 0xFF);
    Wire.write(on_tick  >> 8);
    Wire.write(off_tick & 0xFF);
    Wire.write(off_tick >> 8);
    Wire.endTransmission();
}

void PCA9685::set_pulse_us(uint8_t channel, uint16_t pulse_us) {
    uint16_t off_tick = (uint16_t)(pulse_us / us_per_tick_);
    if (off_tick > PWM_RESOLUTION - 1) off_tick = PWM_RESOLUTION - 1;
    set_pwm(channel, 0, off_tick);
}

void PCA9685::set_angle(uint8_t channel, float angle_deg) {
    // Map angle_deg ∈ [-90, +90] → pulse_us ∈ [500, 2400]
    float clamped = angle_deg;
    if (clamped < SERVO_MIN_DEG) clamped = SERVO_MIN_DEG;
    if (clamped > SERVO_MAX_DEG) clamped = SERVO_MAX_DEG;

    float t = (clamped - SERVO_MIN_DEG) / (SERVO_MAX_DEG - SERVO_MIN_DEG);
    uint16_t pulse_us = (uint16_t)(SERVO_PWM_MIN + t * (SERVO_PWM_MAX - SERVO_PWM_MIN));
    set_pulse_us(channel, pulse_us);
}

void PCA9685::set_all_neutral() {
    for (uint8_t ch = 0; ch < 16; ch++) {
        set_angle(ch, 0.0f);
    }
}

void PCA9685::all_off() {
    Wire.beginTransmission(addr_);
    Wire.write(PCA9685Reg::ALL_LED_OFF_L);
    Wire.write(0x00);
    Wire.write(0x10);   // full-off bit
    Wire.endTransmission();
}
