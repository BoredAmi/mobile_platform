/*
 * Build-time configuration of the jgb_rover RP2040 board (Waveshare RP2040-Zero).
 *
 * Pins and the motor driver type are fixed here. Everything that may need tuning on the robot
 * (controller gains, motor/encoder directions, timeouts) is sent by the host at startup
 * (RL_MSG_SET_CONFIG, values from jgb_rover_description/config/real_hardware.yaml), so it does
 * not need a reflash; the DEFAULT_* values below only apply until then.
 */
#ifndef JGB_CONFIG_H
#define JGB_CONFIG_H

#define FIRMWARE_VERSION 0x0100 /* 1.0 */

/* ---------------- timing ---------------- */
#define CONTROL_HZ 100           /* control loop, encoder + IMU sampling and telemetry rate */
#define WATCHDOG_TIMEOUT_MS 200  /* hardware watchdog: reboot (motors off) if the loop hangs */

/* ---------------- pins (RP2040-Zero edge pins) ----------------
 *  GP0  / GP1   UART0 TX / RX        reserved (debug or a UART link to the Pi later)
 *  GP2  / GP3   left encoder A / B   must be consecutive (PIO), A first
 *  GP4  / GP5   right encoder A / B
 *  GP6  / GP7   I2C1 SDA / SCL       MPU6050 (GY-521 has its own pull-ups)
 *  GP8          MPU6050 INT          not used yet
 *  GP9          motor driver enable  R_EN + L_EN of BOTH BTS7960 modules (10k pull-down to GND)
 *  GP10 / GP11  left / right PWM     (MOTOR_DRIVER_PWM_DIR only) unused with the BTS7960
 *  GP12 / GP13  left  RPWM / LPWM    (IN1 / IN2 in MOTOR_DRIVER_PWM_DIR)
 *  GP14 / GP15  right RPWM / LPWM
 *  GP16         on-board WS2812 status LED
 *  GP26         battery voltage (ADC0) through a divider, see BATTERY_*
 */
#define PIN_LEFT_ENC_A 2
#define PIN_RIGHT_ENC_A 4
#define PIN_I2C_SDA 6
#define PIN_I2C_SCL 7
#define IMU_I2C i2c1
#define PIN_MOTOR_ENABLE 9
#define PIN_LEFT_PWM 10
#define PIN_RIGHT_PWM 11
#define PIN_LEFT_IN1 12
#define PIN_LEFT_IN2 13
#define PIN_RIGHT_IN1 14
#define PIN_RIGHT_IN2 15
#define PIN_STATUS_LED 16
#define PIN_BATTERY_ADC 26

/* ---------------- motor driver ----------------
 * MOTOR_DRIVER_PWM_DIR: speed on PWM, direction on IN1/IN2 (TB6612FNG, L298N with ENA/ENB = PWM).
 *   forward IN1=1 IN2=0, reverse IN1=0 IN2=1, brake IN1=IN2=1, coast IN1=IN2=0.
 * MOTOR_DRIVER_PWM_PWM: PWM on IN1 or IN2, the other low (DRV8833, DRV8871, BTS7960 RPWM/LPWM).
 *   the PWM pins (GP10/11) are unused; brake = both high, coast = both low. */
#define MOTOR_DRIVER_PWM_DIR 1
#define MOTOR_DRIVER_PWM_PWM 2
#define MOTOR_DRIVER MOTOR_DRIVER_PWM_PWM  /* two BTS7960 (IBT-2) modules, one per motor */

#define MOTOR_PWM_FREQ_HZ 20000  /* inaudible; BTS7960 is specified up to 25 kHz. ~1-5 kHz for an L298N */
#define MOTOR_ENABLE_ACTIVE_HIGH 1

/* ---------------- encoders ---------------- */
/* JGB37-520 Hall encoder: power it from 3V3, not 5 V. Its outputs are pulled up to its own supply
 * and the RP2040 pins are NOT 5 V tolerant. The internal pull-ups are enabled as well. */
#define ENCODER_MAX_STEP_RATE 200000  /* PIO sampling sized for this (counts/s); motor max ~7300 */

/* ---------------- IMU (MPU6050 / GY-521) ---------------- */
#define IMU_I2C_ADDR 0x68          /* AD0 low; 0x69 with AD0 high */
#define IMU_I2C_BAUD 400000
#define IMU_DLPF_CFG 3             /* 44 Hz gyro / 42 Hz accel bandwidth, ~5 ms delay */
#define IMU_RETRY_MS 1000          /* re-initialise this often after a failed read */
/* board axes -> imu_link: imu_link.x = sign[0] * board[map[0]] etc. Identity = board X forward,
 * Z up, as in robot_spec.yaml (imu.rpy = 0). Example for a board mounted with X pointing left:
 * map {1, 0, 2}, sign {-1, 1, 1}. */
#define IMU_AXIS_MAP {0, 1, 2}
#define IMU_AXIS_SIGN {1, 1, 1}

/* ---------------- battery (optional) ---------------- */
/* 12 V AGM: e.g. 100k (to battery +) / 22k (to GND) -> 13.8 V reads 2.49 V. 0 = not fitted. */
#define BATTERY_DIVIDER_RATIO 0.0f /* (R1 + R2) / R2, e.g. 5.545f for 100k / 22k */

/* ---------------- status LED ---------------- */
/* WS2812 colour order. Standard WS2812 is GRB; if red and green look swapped on your board, set 0. */
#define STATUS_LED_GRB 1
#define STATUS_LED_BRIGHTNESS 24  /* 0..255, the LED is very bright */

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
