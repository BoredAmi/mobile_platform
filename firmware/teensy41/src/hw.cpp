#include "hw.h"

#include <Arduino.h>
#include <Encoder.h>
#include <IntervalTimer.h>
#include <Wire.h>
#include <math.h>

#include "config.h"

/* ======================= encoders ======================= */

/* Encoder library: x4 decoding on pin-change interrupts (~7300 counts/s per wheel at full speed) */
static Encoder enc_left(PIN_LEFT_ENC_A, PIN_LEFT_ENC_B);
static Encoder enc_right(PIN_RIGHT_ENC_A, PIN_RIGHT_ENC_B);

void encoders_init(void) {
  /* the Encoder constructors already set the pins to INPUT_PULLUP and attached the interrupts */
}

void encoders_read(int32_t counts[2]) {
  counts[0] = enc_left.read();
  counts[1] = enc_right.read();
}

/* ======================= motors ======================= */

static const uint32_t PWM_FULL = 1u << MOTOR_PWM_BITS;  /* analogWrite value for 100 % (always high) */
static const uint8_t in1_pins[2] = {PIN_LEFT_IN1, PIN_RIGHT_IN1};
static const uint8_t in2_pins[2] = {PIN_LEFT_IN2, PIN_RIGHT_IN2};
#if MOTOR_DRIVER == MOTOR_DRIVER_PWM_DIR
static const uint8_t pwm_pins[2] = {PIN_LEFT_PWM, PIN_RIGHT_PWM};
#endif

static void pwm_pin_init(uint8_t pin) {
  pinMode(pin, OUTPUT);
  analogWriteFrequency(pin, MOTOR_PWM_FREQ_HZ);
  analogWrite(pin, 0);
}

static void out_pin_init(uint8_t pin) {
  pinMode(pin, OUTPUT);
  digitalWriteFast(pin, LOW);
}

static uint32_t duty_level(float duty) {
  float m = fabsf(duty);
  if (m > 1.0f) m = 1.0f;
  return (uint32_t)(m * (float)PWM_FULL + 0.5f);
}

void motors_init(void) {
  out_pin_init(PIN_MOTOR_ENABLE);
  digitalWriteFast(PIN_MOTOR_ENABLE, !MOTOR_ENABLE_ACTIVE_HIGH);
  analogWriteResolution(MOTOR_PWM_BITS);
  for (int i = 0; i < 2; ++i) {
#if MOTOR_DRIVER == MOTOR_DRIVER_PWM_DIR
    pwm_pin_init(pwm_pins[i]);
    out_pin_init(in1_pins[i]);
    out_pin_init(in2_pins[i]);
#elif MOTOR_DRIVER == MOTOR_DRIVER_PWM_PWM
    pwm_pin_init(in1_pins[i]);
    pwm_pin_init(in2_pins[i]);
#else
#error "unknown MOTOR_DRIVER"
#endif
  }
}

void motors_set(const float duty[2], bool brake_at_zero) {
  for (int i = 0; i < 2; ++i) {
    uint32_t level = duty_level(duty[i]);
#if MOTOR_DRIVER == MOTOR_DRIVER_PWM_DIR
    if (level == 0) {
      digitalWriteFast(in1_pins[i], brake_at_zero);
      digitalWriteFast(in2_pins[i], brake_at_zero);
    } else {
      digitalWriteFast(in1_pins[i], duty[i] > 0.0f);
      digitalWriteFast(in2_pins[i], duty[i] < 0.0f);
    }
    analogWrite(pwm_pins[i], level);
#else
    if (level == 0) {
      analogWrite(in1_pins[i], brake_at_zero ? PWM_FULL : 0);
      analogWrite(in2_pins[i], brake_at_zero ? PWM_FULL : 0);
    } else if (duty[i] > 0.0f) {
      analogWrite(in2_pins[i], 0);
      analogWrite(in1_pins[i], level);
    } else {
      analogWrite(in1_pins[i], 0);
      analogWrite(in2_pins[i], level);
    }
#endif
  }
}

void motors_enable(bool on) {
  if (!on) {
    const float zero[2] = {0.0f, 0.0f};
    motors_set(zero, false);
  }
  digitalWriteFast(PIN_MOTOR_ENABLE, on == MOTOR_ENABLE_ACTIVE_HIGH);
}

/* ======================= MPU6050 ======================= */

#define MPU_SMPLRT_DIV 0x19
#define MPU_CONFIG 0x1A
#define MPU_GYRO_CONFIG 0x1B
#define MPU_ACCEL_CONFIG 0x1C
#define MPU_ACCEL_XOUT_H 0x3B
#define MPU_PWR_MGMT_1 0x6B
#define MPU_WHO_AM_I 0x75

#define GYRO_LSB_PER_DPS 131.0f    /* +-250 dps (robot_spec imu.gyro_range_dps) */
#define ACCEL_LSB_PER_G 16384.0f   /* +-2 g (robot_spec imu.accel_range_g) */
#define G_TO_MS2 9.80665f
#define DEG_TO_RAD_F 0.01745329252f

static const int axis_map[3] = IMU_AXIS_MAP;
static const float axis_sign[3] = IMU_AXIS_SIGN;
static bool wire_started;

static bool mpu_write(uint8_t reg, uint8_t value) {
  Wire.beginTransmission(IMU_I2C_ADDR);
  Wire.write(reg);
  Wire.write(value);
  return Wire.endTransmission() == 0;
}

static bool mpu_read(uint8_t reg, uint8_t *buf, size_t len) {
  Wire.beginTransmission(IMU_I2C_ADDR);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  if (Wire.requestFrom((uint8_t)IMU_I2C_ADDR, (uint8_t)len) != len) {
    return false;
  }
  for (size_t i = 0; i < len; ++i) {
    buf[i] = (uint8_t)Wire.read();
  }
  return true;
}

/* A device reset mid-transfer can leave SDA held low; clock it free (9 pulses + STOP). */
static void i2c_bus_recover(void) {
  pinMode(PIN_I2C_SDA, INPUT_PULLUP);
  pinMode(PIN_I2C_SCL, INPUT_PULLUP);
  for (int i = 0; i < 9 && !digitalReadFast(PIN_I2C_SDA); ++i) {
    pinMode(PIN_I2C_SCL, OUTPUT);
    digitalWriteFast(PIN_I2C_SCL, LOW);
    delayMicroseconds(5);
    pinMode(PIN_I2C_SCL, INPUT_PULLUP);  /* released: pulled high */
    delayMicroseconds(5);
  }
  /* STOP: SDA low -> high while SCL high */
  pinMode(PIN_I2C_SDA, OUTPUT);
  digitalWriteFast(PIN_I2C_SDA, LOW);
  delayMicroseconds(5);
  pinMode(PIN_I2C_SDA, INPUT_PULLUP);
  delayMicroseconds(5);
}

uint8_t imu_init(void) {
  if (wire_started) {
    Wire.end();
  }
  i2c_bus_recover();
  Wire.begin();
  Wire.setClock(IMU_I2C_BAUD);
  wire_started = true;

  uint8_t who = 0;
  if (!mpu_read(MPU_WHO_AM_I, &who, 1)) {
    return 0;
  }
  /* reset, then clock from the X gyro PLL (more stable than the internal oscillator) */
  if (!mpu_write(MPU_PWR_MGMT_1, 0x80)) return 0;
  delay(100);
  bool ok = mpu_write(MPU_PWR_MGMT_1, 0x01) &&
            mpu_write(MPU_SMPLRT_DIV, 0x00) &&      /* 1 kHz internal rate with the DLPF on */
            mpu_write(MPU_CONFIG, IMU_DLPF_CFG) &&
            mpu_write(MPU_GYRO_CONFIG, 0x00) &&     /* +-250 dps */
            mpu_write(MPU_ACCEL_CONFIG, 0x00);      /* +-2 g */
  if (!ok) return 0;
  delay(20);
  return who ? who : 0xFF;  /* a 0 register value still means "answered" */
}

bool imu_read(imu_sample_t *out) {
  uint8_t b[14];
  if (!mpu_read(MPU_ACCEL_XOUT_H, b, sizeof(b))) {
    return false;
  }
  float board_a[3], board_g[3];
  for (int i = 0; i < 3; ++i) {
    int16_t a = (int16_t)((b[2 * i] << 8) | b[2 * i + 1]);
    int16_t g = (int16_t)((b[8 + 2 * i] << 8) | b[8 + 2 * i + 1]);
    board_a[i] = (float)a / ACCEL_LSB_PER_G * G_TO_MS2;
    board_g[i] = (float)g / GYRO_LSB_PER_DPS * DEG_TO_RAD_F;
  }
  for (int i = 0; i < 3; ++i) {
    out->accel[i] = axis_sign[i] * board_a[axis_map[i]];
    out->gyro[i] = axis_sign[i] * board_g[axis_map[i]];
  }
  int16_t t = (int16_t)((b[6] << 8) | b[7]);
  out->temp_c = (float)t / 340.0f + 36.53f;
  return true;
}

/* ======================= battery ======================= */

void battery_init(void) {
  analogReadResolution(12);
}

uint16_t battery_read_mv(void) {
  if (!(BATTERY_DIVIDER_RATIO > 0.0f)) {
    return 0;
  }
  uint32_t sum = 0;
  for (int i = 0; i < 8; ++i) {
    sum += analogRead(PIN_BATTERY_ADC);
  }
  float volts = (float)sum / 8.0f * (3.3f / 4096.0f) * BATTERY_DIVIDER_RATIO;
  return (uint16_t)(volts * 1000.0f + 0.5f);
}

/* ======================= status LED ======================= */

void status_led_init(void) {
  pinMode(PIN_STATUS_LED, OUTPUT);
  digitalWriteFast(PIN_STATUS_LED, LOW);
}

void status_led(bool on) {
  digitalWriteFast(PIN_STATUS_LED, on);
}

/* ======================= safety ======================= */

void watchdog_start(void) {
  CCM_CCGR3 |= CCM_CCGR3_WDOG1(3);  /* clock on */
  WDOG1_WMCR = 0;                    /* power-down counter off */
  /* WT: timeout = (WT + 1) * 0.5 s. SRS / WDA = 1: no software reset, no WDOG_B pin assertion.
   * WDE can only be set once: the watchdog then runs until the next reset. */
  WDOG1_WCR = WDOG_WCR_WT(WATCHDOG_TIMEOUT_HALF_S - 1) | WDOG_WCR_SRS | WDOG_WCR_WDA | WDOG_WCR_WDE;
  watchdog_feed();
}

void watchdog_feed(void) {
  WDOG1_WSR = 0x5555;
  WDOG1_WSR = 0xAAAA;
}

bool reset_was_watchdog(void) {
  uint32_t srsr = SRC_SRSR;
  SRC_SRSR = srsr;  /* sticky bits, write 1 to clear: the next boot reports its own cause */
  return srsr & (SRC_SRSR_WDOG_RST_B | SRC_SRSR_WDOG3_RST_B);
}

static IntervalTimer guard_timer;
static volatile uint32_t last_alive_ms;

static void guard_isr(void) {
  if (millis() - last_alive_ms > LOOP_STALL_MS) {
    /* the main loop is stuck (I2C hang, bug): driver off now, the hardware watchdog follows */
    digitalWriteFast(PIN_MOTOR_ENABLE, !MOTOR_ENABLE_ACTIVE_HIGH);
#if MOTOR_DRIVER == MOTOR_DRIVER_PWM_DIR
    analogWrite(PIN_LEFT_PWM, 0);
    analogWrite(PIN_RIGHT_PWM, 0);
#else
    for (int i = 0; i < 2; ++i) {
      analogWrite(in1_pins[i], 0);
      analogWrite(in2_pins[i], 0);
    }
#endif
  }
}

void loop_guard_start(void) {
  last_alive_ms = millis();
  guard_timer.priority(64);  /* above USB and pin interrupts' default 128 */
  guard_timer.begin(guard_isr, 10000);
}

void loop_alive(void) {
  last_alive_ms = millis();
}

/* ======================= misc ======================= */

void board_unique_id(uint8_t id[8]) {
  uint32_t lo = HW_OCOTP_CFG0, hi = HW_OCOTP_CFG1;
  for (int i = 0; i < 4; ++i) {
    id[i] = (uint8_t)(hi >> (24 - 8 * i));
    id[4 + i] = (uint8_t)(lo >> (24 - 8 * i));
  }
}

void reboot_to_bootloader(void) {
  motors_enable(false);
  delay(10);
  _reboot_Teensyduino_();
}
