#include "hw.h"

#include <Arduino.h>
#include <ESP32Encoder.h>
#include <Wire.h>
#include <esp_system.h>
#include <esp_task_wdt.h>
#include <esp_timer.h>
#include <math.h>

#include "config.h"

/* ======================= encoders ======================= */

/* PCNT hardware counters: x4 decoding without CPU load */
static ESP32Encoder enc_left;
static ESP32Encoder enc_right;

void encoders_init(void) {
  ESP32Encoder::useInternalWeakPullResistors = puType::up;
  enc_left.attachFullQuad(PIN_LEFT_ENC_A, PIN_LEFT_ENC_B);
  enc_right.attachFullQuad(PIN_RIGHT_ENC_A, PIN_RIGHT_ENC_B);
  enc_left.setFilter(ENCODER_FILTER);
  enc_right.setFilter(ENCODER_FILTER);
  enc_left.clearCount();
  enc_right.clearCount();
}

void encoders_read(int32_t counts[2]) {
  counts[0] = (int32_t)enc_left.getCount();
  counts[1] = (int32_t)enc_right.getCount();
}

/* ======================= motors ======================= */

/* LEDC duty for 100 % (the counter never reaches 2^bits, so full on is the largest duty) */
static const uint32_t PWM_FULL = (1u << MOTOR_PWM_BITS) - 1;
static const uint8_t in1_pins[2] = {PIN_LEFT_IN1, PIN_RIGHT_IN1};
static const uint8_t in2_pins[2] = {PIN_LEFT_IN2, PIN_RIGHT_IN2};
static const uint8_t in1_channels[2] = {0, 2};
static const uint8_t in2_channels[2] = {1, 3};

#if ESP_ARDUINO_VERSION_MAJOR >= 3
static void pwm_init(uint8_t pin, uint8_t channel) {
  ledcAttachChannel(pin, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_BITS, channel);
  ledcWrite(pin, 0);
}
static void pwm_write(uint8_t pin, uint8_t channel, uint32_t level) {
  ledcWrite(pin, level);
}
#else
static void pwm_init(uint8_t pin, uint8_t channel) {
  ledcSetup(channel, MOTOR_PWM_FREQ_HZ, MOTOR_PWM_BITS);
  ledcAttachPin(pin, channel);
  ledcWrite(channel, 0);
}
static void pwm_write(uint8_t pin, uint8_t channel, uint32_t level) {
  ledcWrite(channel, level);
}
#endif

static uint32_t duty_level(float duty) {
  float m = fabsf(duty);
  if (m > 1.0f) m = 1.0f;
  return (uint32_t)(m * (float)PWM_FULL + 0.5f);
}

void motors_init(void) {
  pinMode(PIN_MOTOR_ENABLE, OUTPUT);
  digitalWrite(PIN_MOTOR_ENABLE, !MOTOR_ENABLE_ACTIVE_HIGH);
  for (int i = 0; i < 2; ++i) {
    pwm_init(in1_pins[i], in1_channels[i]);
    pwm_init(in2_pins[i], in2_channels[i]);
  }
}

void motors_set(const float duty[2], bool brake_at_zero) {
  for (int i = 0; i < 2; ++i) {
    uint32_t level = duty_level(duty[i]);
    if (level == 0) {
      uint32_t idle = brake_at_zero ? PWM_FULL : 0;
      pwm_write(in1_pins[i], in1_channels[i], idle);
      pwm_write(in2_pins[i], in2_channels[i], idle);
    } else if (duty[i] > 0.0f) {
      pwm_write(in2_pins[i], in2_channels[i], 0);
      pwm_write(in1_pins[i], in1_channels[i], level);
    } else {
      pwm_write(in1_pins[i], in1_channels[i], 0);
      pwm_write(in2_pins[i], in2_channels[i], level);
    }
  }
}

void motors_enable(bool on) {
  if (!on) {
    const float zero[2] = {0.0f, 0.0f};
    motors_set(zero, false);
  }
  digitalWrite(PIN_MOTOR_ENABLE, on == MOTOR_ENABLE_ACTIVE_HIGH);
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
  for (int i = 0; i < 9 && !digitalRead(PIN_I2C_SDA); ++i) {
    pinMode(PIN_I2C_SCL, OUTPUT);
    digitalWrite(PIN_I2C_SCL, LOW);
    delayMicroseconds(5);
    pinMode(PIN_I2C_SCL, INPUT_PULLUP);  /* released: pulled high */
    delayMicroseconds(5);
  }
  /* STOP: SDA low -> high while SCL high */
  pinMode(PIN_I2C_SDA, OUTPUT);
  digitalWrite(PIN_I2C_SDA, LOW);
  delayMicroseconds(5);
  pinMode(PIN_I2C_SDA, INPUT_PULLUP);
  delayMicroseconds(5);
}

uint8_t imu_init(void) {
  if (wire_started) {
    Wire.end();
  }
  i2c_bus_recover();
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(IMU_I2C_BAUD);
  Wire.setTimeOut(IMU_I2C_TIMEOUT_MS);
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
    sum += analogReadMilliVolts(PIN_BATTERY_ADC);  /* factory-calibrated, ADC1 */
  }
  float volts = (float)sum / 8.0f / 1000.0f * BATTERY_DIVIDER_RATIO;
  return (uint16_t)(volts * 1000.0f + 0.5f);
}

/* ======================= status LED ======================= */

void status_led_init(void) {
  pinMode(PIN_STATUS_LED, OUTPUT);
  digitalWrite(PIN_STATUS_LED, LOW);
}

void status_led(bool on) {
  digitalWrite(PIN_STATUS_LED, on);
}

/* ======================= safety ======================= */

void watchdog_start(void) {
#if ESP_ARDUINO_VERSION_MAJOR >= 3
  esp_task_wdt_config_t cfg = {};
  cfg.timeout_ms = WATCHDOG_TIMEOUT_S * 1000;
  cfg.idle_core_mask = 0;
  cfg.trigger_panic = true;
  esp_task_wdt_reconfigure(&cfg);
#else
  esp_task_wdt_init(WATCHDOG_TIMEOUT_S, true);  /* true: reset the chip on timeout */
#endif
  esp_task_wdt_add(NULL);                       /* the Arduino loop task */
}

void watchdog_feed(void) {
  esp_task_wdt_reset();
}

bool reset_was_watchdog(void) {
  esp_reset_reason_t r = esp_reset_reason();
  return r == ESP_RST_TASK_WDT || r == ESP_RST_INT_WDT || r == ESP_RST_WDT || r == ESP_RST_PANIC;
}

static esp_timer_handle_t guard_timer;
static volatile uint32_t last_alive_ms;

static void guard_cb(void *arg) {
  if (millis() - last_alive_ms > LOOP_STALL_MS) {
    /* the main loop is stuck (I2C hang, bug): driver off now, the watchdog follows */
    digitalWrite(PIN_MOTOR_ENABLE, !MOTOR_ENABLE_ACTIVE_HIGH);
    for (int i = 0; i < 2; ++i) {
      pwm_write(in1_pins[i], in1_channels[i], 0);
      pwm_write(in2_pins[i], in2_channels[i], 0);
    }
  }
}

void loop_guard_start(void) {
  last_alive_ms = millis();
  esp_timer_create_args_t args = {};
  args.callback = guard_cb;
  args.name = "loop_guard";
  esp_timer_create(&args, &guard_timer);
  esp_timer_start_periodic(guard_timer, 10000);
}

void loop_alive(void) {
  last_alive_ms = millis();
}

/* ======================= misc ======================= */

void board_unique_id(uint8_t id[8]) {
  uint64_t mac = ESP.getEfuseMac();
  for (int i = 0; i < 8; ++i) {
    id[i] = (uint8_t)(mac >> (56 - 8 * i));
  }
}

void reboot_to_bootloader(void) {
  motors_enable(false);
  delay(10);
  esp_restart();  /* uploads put the board into its bootloader through DTR / RTS, not by command */
}
