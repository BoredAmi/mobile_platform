/*
 * Build-time configuration of the jgb_rover motor / IMU board (Teensy 4.1).
 *
 * Pins and the motor driver type are fixed here. Everything that may need tuning on the robot
 * (controller gains, motor/encoder directions, timeouts) is sent by the host at startup
 * (RL_MSG_SET_CONFIG, values from jgb_rover_description/config/real_hardware.yaml), so it does
 * not need a reflash; the DEFAULT_* values below only apply until then.
 */
#ifndef JGB_CONFIG_H
#define JGB_CONFIG_H

#define FIRMWARE_VERSION 0x0200 /* 2.0: Teensy 4.1 */

/* ---------------- timing ---------------- */
#define CONTROL_HZ 100           /* control loop, encoder + IMU sampling and telemetry rate */
#define LOOP_STALL_MS 150        /* timer interrupt switches the motors off if the loop stalls this long */
#define WATCHDOG_TIMEOUT_HALF_S 1 /* hardware watchdog (WDOG1) resets the chip after 0.5 s * this */

/* ---------------- pins (Teensy 4.1) ----------------
 *  0  / 1       Serial1 RX / TX      reserved (UART link to the Pi later)
 *  20 / 3       left encoder A / B   interrupt pins (Encoder library), any digital pin works
 *  2            NOT USED: damaged on this board
 *  4  / 5       right encoder A / B
 *  6  / 7       left  RPWM / LPWM    BTS7960 (IN1 / IN2 with MOTOR_DRIVER_PWM_DIR)
 *  8  / 9       right RPWM / LPWM
 *  10           motor driver enable  R_EN + L_EN of BOTH BTS7960 modules (10k pull-down to GND)
 *  11 / 12      left / right PWM     only with MOTOR_DRIVER_PWM_DIR
 *  13           on-board LED         status
 *  14 (A0)      battery voltage through a divider, see BATTERY_*
 *  18 / 19      SDA / SCL (Wire)     MPU6050 (GY-521 has its own pull-ups)
 *  22           MPU6050 INT          not used yet
 */
#define PIN_LEFT_ENC_A 20  /* pin 2 is damaged on this board */
#define PIN_LEFT_ENC_B 3
#define PIN_RIGHT_ENC_A 4
#define PIN_RIGHT_ENC_B 5
#define PIN_LEFT_IN1 6
#define PIN_LEFT_IN2 7
#define PIN_RIGHT_IN1 8
#define PIN_RIGHT_IN2 9
#define PIN_MOTOR_ENABLE 10
#define PIN_LEFT_PWM 11
#define PIN_RIGHT_PWM 12
#define PIN_STATUS_LED 13
#define PIN_BATTERY_ADC 14
#define PIN_I2C_SDA 18
#define PIN_I2C_SCL 19

/* ---------------- motor driver ----------------
 * MOTOR_DRIVER_PWM_PWM: PWM on IN1 (RPWM) or IN2 (LPWM), the other low: BTS7960 / IBT-2 modules,
 *   DRV8833, DRV8871. Brake = both high, coast = both low.
 * MOTOR_DRIVER_PWM_DIR: speed on PWM, direction on IN1/IN2 (TB6612FNG, L298N with ENA/ENB = PWM).
 *   forward IN1=1 IN2=0, reverse IN1=0 IN2=1, brake IN1=IN2=1, coast IN1=IN2=0. */
#define MOTOR_DRIVER_PWM_DIR 1
#define MOTOR_DRIVER_PWM_PWM 2
#define MOTOR_DRIVER MOTOR_DRIVER_PWM_PWM  /* two BTS7960 (IBT-2) modules, one per motor */

#define MOTOR_PWM_FREQ_HZ 20000  /* inaudible; BTS7960 is specified up to 25 kHz. ~1-5 kHz for an L298N */
#define MOTOR_PWM_BITS 12
#define MOTOR_ENABLE_ACTIVE_HIGH 1

/* ---------------- encoders ---------------- */
/* JGB37-520 Hall encoder: power it from 3.3 V, not 5 V. Its outputs are pulled up to its own
 * supply and the Teensy 4.1 pins are NOT 5 V tolerant. The internal pull-ups are enabled too. */

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
