/*
 * Build-time configuration of the jgb_rover motor / IMU board (classic ESP32 dev board).
 *
 * Same job as firmware/teensy41/src/config.h: pins and the motor driver type are fixed here;
 * everything tunable (gains, directions, timeouts) is sent by the host at startup
 * (RL_MSG_SET_CONFIG, values from jgb_rover_description/config/real_hardware.yaml).
 */
#ifndef JGB_CONFIG_H
#define JGB_CONFIG_H

#define FIRMWARE_VERSION 0x0300 /* 3.0: ESP32 (2.0 = Teensy 4.1) */

/* ---------------- timing ---------------- */
#define CONTROL_HZ 100            /* control loop, encoder + IMU sampling and telemetry rate */
#define LOOP_STALL_MS 150         /* timer task switches the motors off if the loop stalls this long */
#define WATCHDOG_TIMEOUT_S 1      /* task watchdog resets the chip after this many seconds */

/* ---------------- host link ----------------
 * UART0 through the board's USB-UART chip (CP210x / CH340). The host must use the same rate
 * (real_hardware.yaml mcu.baud_rate). Telemetry is ~9 KB/s, so 115200 baud is too slow. */
#define LINK_BAUD 921600

/* ---------------- pins (ESP32-WROOM-32 DevKit) ----------------
 *  1 / 3        UART0 TX / RX        host link, do not use
 *  6 - 11       flash                never use
 *  0, 2, 5, 12, 15  strapping pins   avoided, except GPIO2 as the status LED output
 *  34 - 39      input only, no pull-ups
 *  32 / 33      left encoder A / B   PCNT hardware counter
 *  25 / 26      right encoder A / B
 *  16 / 17      left  RPWM / LPWM    BTS7960 (not on WROVER modules: GPIO16/17 are PSRAM there)
 *  18 / 19      right RPWM / LPWM
 *  23           motor driver enable  R_EN + L_EN of BOTH BTS7960 modules (10k pull-down to GND)
 *  2            on-board LED         status
 *  34           battery voltage through a divider, see BATTERY_*
 *  21 / 22      SDA / SCL (Wire)     MPU6050 (GY-521 has its own pull-ups)
 */
#define PIN_LEFT_ENC_A 32
#define PIN_LEFT_ENC_B 33
#define PIN_RIGHT_ENC_A 25
#define PIN_RIGHT_ENC_B 26
#define PIN_LEFT_IN1 16
#define PIN_LEFT_IN2 17
#define PIN_RIGHT_IN1 18
#define PIN_RIGHT_IN2 19
#define PIN_MOTOR_ENABLE 23
#define PIN_STATUS_LED 2
#define PIN_BATTERY_ADC 34
#define PIN_I2C_SDA 21
#define PIN_I2C_SCL 22

/* ---------------- motor driver ----------------
 * Two BTS7960 (IBT-2) modules, one per motor: PWM on IN1 (RPWM) or IN2 (LPWM), the other low.
 * Brake = both high, coast = both low. (The Teensy firmware also has a PWM + direction mode; not
 * needed here.) */
#define MOTOR_PWM_FREQ_HZ 20000   /* inaudible; BTS7960 is specified up to 25 kHz */
#define MOTOR_PWM_BITS 10         /* LEDC: 20 kHz x 2^bits must stay below the 80 MHz timer clock */
#define MOTOR_ENABLE_ACTIVE_HIGH 1

/* ---------------- encoders ---------------- */
/* Power them from 3.3 V (ESP32 pins are not 5 V tolerant). Hardware pulse counter, x4 decoding.
 * Pulses shorter than ENCODER_FILTER APB cycles (80 MHz) are ignored: 1023 = 12.8 us, far below
 * the ~140 us between edges at full speed. */
#define ENCODER_FILTER 1023

/* ---------------- IMU (MPU6050 / GY-521) ---------------- */
#define IMU_I2C_ADDR 0x68          /* AD0 low; 0x69 with AD0 high */
#define IMU_I2C_BAUD 400000
#define IMU_I2C_TIMEOUT_MS 5       /* a hung bus must not stall the control loop for long */
#define IMU_DLPF_CFG 3             /* 44 Hz gyro / 42 Hz accel bandwidth, ~5 ms delay */
#define IMU_RETRY_MS 1000          /* re-initialise this often after a failed read */
/* board axes -> imu_link: imu_link.x = sign[0] * board[map[0]] etc. Identity = board X forward,
 * Z up, as in robot_spec.yaml (imu.rpy = 0). */
#define IMU_AXIS_MAP {0, 1, 2}
#define IMU_AXIS_SIGN {1, 1, 1}

/* ---------------- battery (optional) ---------------- */
/* 12 V AGM: 100k (to battery +) / 22k (to GND) -> 13.8 V reads 2.49 V. 0 = not fitted. */
#define BATTERY_DIVIDER_RATIO 0.0f /* (R1 + R2) / R2, e.g. 5.545f for 100k / 22k */

/* ---------------- defaults until the host sends its config ---------------- */
#define DEFAULT_COUNTS_PER_REV 3960.0f /* robot_spec.yaml encoders.counts_per_output_rev_x4 */
#define DEFAULT_KP 0.05f
#define DEFAULT_KI 0.3f
#define DEFAULT_KFF 0.085f             /* ~ 1 / 11.52 rad/s no-load speed at 12 V */
#define DEFAULT_DEADBAND_DUTY 0.04f
#define DEFAULT_MAX_DUTY 0.95f
#define DEFAULT_CMD_TIMEOUT_MS 250
#define DEFAULT_VEL_WINDOW 4           /* 40 ms at 100 Hz: 0.04 rad/s resolution */
#define DEFAULT_CFG_FLAGS (RL_CFG_INVERT_RIGHT_MOTOR | RL_CFG_INVERT_RIGHT_ENCODER | RL_CFG_IDLE_BRAKE)

#endif /* JGB_CONFIG_H */
