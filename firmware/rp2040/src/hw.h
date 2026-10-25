/* RP2040 peripherals: encoders (PIO), motor driver (PWM), MPU6050 (I2C), battery ADC, status LED. */
#ifndef JGB_HW_H
#define JGB_HW_H

#include <stdbool.h>
#include <stdint.h>

/* ---- encoders: PIO0 SM0 = left, SM1 = right; raw counts, signs applied by the caller ---- */
void encoders_init(void);
void encoders_read(int32_t counts[2]);

/* ---- motors ---- */
void motors_init(void);
/* duty -1..1 per wheel ([left, right], already direction-corrected); 0 = brake or coast */
void motors_set(const float duty[2], bool brake_at_zero);
/* enable pin; disabling also zeroes both outputs (coast) */
void motors_enable(bool on);

/* ---- MPU6050 ---- */
typedef struct {
  float gyro[3];   /* rad/s, imu_link */
  float accel[3];  /* m/s^2, imu_link */
  float temp_c;
} imu_sample_t;

/* Reset and configure; returns WHO_AM_I (0 = no device answered). */
uint8_t imu_init(void);
bool imu_read(imu_sample_t *out);

/* ---- battery: millivolts, 0 when BATTERY_DIVIDER_RATIO is 0 ---- */
void battery_init(void);
uint16_t battery_read_mv(void);

/* ---- status LED (on-board WS2812) ---- */
void status_led_init(void);
void status_led_set(uint8_t r, uint8_t g, uint8_t b);

#endif /* JGB_HW_H */
