/* ESP32 peripherals: encoders (PCNT), motor driver (LEDC PWM), MPU6050 (I2C), battery ADC, LED,
 * watchdogs. Same interface as firmware/teensy41/src/hw.h. */
#ifndef JGB_HW_H
#define JGB_HW_H

#include <stdbool.h>
#include <stdint.h>

/* ---- encoders: raw counts, signs applied by the caller ---- */
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

/* ---- status LED ---- */
void status_led_init(void);
void status_led(bool on);

/* ---- safety ----
 * Task watchdog: resets the chip unless watchdog_feed() runs in time.
 * Loop guard: a 100 Hz timer task switches the motor driver off when loop_alive() was not called
 * for LOOP_STALL_MS, long before the watchdog fires. */
void watchdog_start(void);
void watchdog_feed(void);
bool reset_was_watchdog(void);  /* read once at boot */
void loop_guard_start(void);
void loop_alive(void);

/* ---- misc ---- */
void board_unique_id(uint8_t id[8]);
void reboot_to_bootloader(void);  /* ESP32: only restarts, uploads reset the board through DTR / RTS */

#endif /* JGB_HW_H */
