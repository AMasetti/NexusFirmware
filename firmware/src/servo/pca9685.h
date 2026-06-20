#pragma once
#include <stdint.h>

// ─── PCA9685 PWM driver ───────────────────────────────────────────────────────
// Controls up to 16 MG995 servos via I2C PWM driver.
// PWM frequency: 50 Hz → period = 20 ms
// MG995 pulse range: 500–2400 µs → 0–180°

class PCA9685 {
public:
    PCA9685();

    // Initialize. Returns false if device not found.
    bool begin(uint8_t addr, uint32_t pwm_freq_hz);

    // Set a channel to a given angle in degrees (-90 to +90 mapped to 0-180).
    // angle_deg: physical angle in degrees, centered at 0 = 90° servo midpoint
    void set_angle(uint8_t channel, float angle_deg);

    // Set a channel directly by pulse width in microseconds
    void set_pulse_us(uint8_t channel, uint16_t pulse_us);

    // Set all channels to their neutral position (90°)
    void set_all_neutral();

    // Disable all outputs (set to 0)
    void all_off();

private:
    uint8_t  addr_;
    uint32_t osc_freq_;      // internal oscillator frequency [Hz]
    float    us_per_tick_;   // microseconds per PWM tick

    void write_reg(uint8_t reg, uint8_t value);
    uint8_t read_reg(uint8_t reg);
    void set_pwm(uint8_t channel, uint16_t on_tick, uint16_t off_tick);

    static constexpr uint32_t INTERNAL_OSC_HZ = 25000000UL;
    static constexpr uint16_t PWM_RESOLUTION   = 4096;
};

// PCA9685 register map
namespace PCA9685Reg {
    constexpr uint8_t MODE1      = 0x00;
    constexpr uint8_t MODE2      = 0x01;
    constexpr uint8_t PRESCALE   = 0xFE;
    constexpr uint8_t LED0_ON_L  = 0x06;   // channel 0 base; +4 per channel
    constexpr uint8_t ALL_LED_ON_L  = 0xFA;
    constexpr uint8_t ALL_LED_OFF_L = 0xFC;
}
