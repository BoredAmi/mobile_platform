#include "hw.h"

#include <math.h>

#include "config.h"
#include "hardware/adc.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"
#include "quadrature_encoder.pio.h"
#include "ws2812.pio.h"

/* ======================= encoders ======================= */

#define ENC_PIO pio0

void encoders_init(void) {
  /* the program uses computed jumps and must sit at offset 0; both SMs share it */
  pio_add_program_at_offset(ENC_PIO, &quadrature_encoder_program, 0);
  quadrature_encoder_program_init(ENC_PIO, 0, PIN_LEFT_ENC_A, ENCODER_MAX_STEP_RATE);
  quadrature_encoder_program_init(ENC_PIO, 1, PIN_RIGHT_ENC_A, ENCODER_MAX_STEP_RATE);
}

void encoders_read(int32_t counts[2]) {
  counts[0] = quadrature_encoder_get_count(ENC_PIO, 0);
  counts[1] = quadrature_encoder_get_count(ENC_PIO, 1);
}

/* ======================= motors ======================= */

static uint16_t pwm_top;

static void pwm_pin_init(uint pin) {
  gpio_set_function(pin, GPIO_FUNC_PWM);
  uint slice = pwm_gpio_to_slice_num(pin);
  pwm_config c = pwm_get_default_config();
  uint32_t sys = clock_get_hz(clk_sys);
  /* top = sys / freq - 1 with clkdiv 1 (6249 at 125 MHz / 20 kHz); divide for low frequencies */
  float div = 1.0f;
  uint32_t top = sys / MOTOR_PWM_FREQ_HZ - 1;
  while (top > 65535) {
    div *= 2.0f;
    top = (uint32_t)(sys / (div * MOTOR_PWM_FREQ_HZ)) - 1;
  }
  pwm_top = (uint16_t)top;
  pwm_config_set_clkdiv(&c, div);
  pwm_config_set_wrap(&c, pwm_top);
  pwm_init(slice, &c, true);
  pwm_set_gpio_level(pin, 0);
}

static void out_pin_init(uint pin) {
  gpio_init(pin);
  gpio_set_dir(pin, GPIO_OUT);
  gpio_put(pin, 0);
}

static uint16_t duty_level(float duty) {
  float m = fabsf(duty);
  if (m > 1.0f) m = 1.0f;
  return (uint16_t)(m * (float)(pwm_top + 1) + 0.5f);
}

#if MOTOR_DRIVER == MOTOR_DRIVER_PWM_DIR
static const uint pwm_pins[2] = {PIN_LEFT_PWM, PIN_RIGHT_PWM};
#endif
static const uint in1_pins[2] = {PIN_LEFT_IN1, PIN_RIGHT_IN1};
static const uint in2_pins[2] = {PIN_LEFT_IN2, PIN_RIGHT_IN2};

void motors_init(void) {
  out_pin_init(PIN_MOTOR_ENABLE);
  gpio_put(PIN_MOTOR_ENABLE, !MOTOR_ENABLE_ACTIVE_HIGH);
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
    uint16_t level = duty_level(duty[i]);
#if MOTOR_DRIVER == MOTOR_DRIVER_PWM_DIR
    if (level == 0) {
      gpio_put(in1_pins[i], brake_at_zero);
      gpio_put(in2_pins[i], brake_at_zero);
    } else {
      gpio_put(in1_pins[i], duty[i] > 0.0f);
      gpio_put(in2_pins[i], duty[i] < 0.0f);
    }
    pwm_set_gpio_level(pwm_pins[i], level);
#else
    uint16_t full = (uint16_t)(pwm_top + 1);
    if (level == 0) {
      pwm_set_gpio_level(in1_pins[i], brake_at_zero ? full : 0);
      pwm_set_gpio_level(in2_pins[i], brake_at_zero ? full : 0);
    } else if (duty[i] > 0.0f) {
      pwm_set_gpio_level(in1_pins[i], level);
      pwm_set_gpio_level(in2_pins[i], 0);
    } else {
      pwm_set_gpio_level(in1_pins[i], 0);
      pwm_set_gpio_level(in2_pins[i], level);
    }
#endif
  }
}

void motors_enable(bool on) {
  if (!on) {
    const float zero[2] = {0.0f, 0.0f};
    motors_set(zero, false);
  }
  gpio_put(PIN_MOTOR_ENABLE, on == MOTOR_ENABLE_ACTIVE_HIGH);
}

/* ======================= MPU6050 ======================= */

#define MPU_SMPLRT_DIV 0x19
#define MPU_CONFIG 0x1A
#define MPU_GYRO_CONFIG 0x1B
#define MPU_ACCEL_CONFIG 0x1C
#define MPU_ACCEL_XOUT_H 0x3B
#define MPU_PWR_MGMT_1 0x6B
#define MPU_WHO_AM_I 0x75

#define I2C_TIMEOUT_US 2000
#define GYRO_LSB_PER_DPS 131.0f    /* +-250 dps (robot_spec imu.gyro_range_dps) */
#define ACCEL_LSB_PER_G 16384.0f   /* +-2 g (robot_spec imu.accel_range_g) */
#define G_TO_MS2 9.80665f
#define DEG_TO_RAD 0.01745329252f

static const int axis_map[3] = IMU_AXIS_MAP;
static const float axis_sign[3] = IMU_AXIS_SIGN;

static bool mpu_write(uint8_t reg, uint8_t value) {
  uint8_t buf[2] = {reg, value};
  return i2c_write_timeout_us(IMU_I2C, IMU_I2C_ADDR, buf, 2, false, I2C_TIMEOUT_US) == 2;
}

static bool mpu_read(uint8_t reg, uint8_t *buf, size_t len) {
  if (i2c_write_timeout_us(IMU_I2C, IMU_I2C_ADDR, &reg, 1, true, I2C_TIMEOUT_US) != 1) {
    return false;
  }
  return i2c_read_timeout_us(IMU_I2C, IMU_I2C_ADDR, buf, len, false, I2C_TIMEOUT_US * 4) == (int)len;
}

/* A device reset mid-transfer can leave SDA held low; clock it free (9 pulses + STOP). */
static void i2c_bus_recover(void) {
  gpio_init(PIN_I2C_SCL);
  gpio_init(PIN_I2C_SDA);
  gpio_pull_up(PIN_I2C_SCL);
  gpio_pull_up(PIN_I2C_SDA);
  gpio_set_dir(PIN_I2C_SDA, GPIO_IN);
  for (int i = 0; i < 9 && !gpio_get(PIN_I2C_SDA); ++i) {
    gpio_set_dir(PIN_I2C_SCL, GPIO_OUT);
    gpio_put(PIN_I2C_SCL, 0);
    sleep_us(5);
    gpio_set_dir(PIN_I2C_SCL, GPIO_IN);  /* released: pulled high */
    sleep_us(5);
  }
  /* STOP: SDA low -> high while SCL high */
  gpio_set_dir(PIN_I2C_SDA, GPIO_OUT);
  gpio_put(PIN_I2C_SDA, 0);
  sleep_us(5);
  gpio_set_dir(PIN_I2C_SDA, GPIO_IN);
  sleep_us(5);
}

uint8_t imu_init(void) {
  i2c_bus_recover();
  i2c_init(IMU_I2C, IMU_I2C_BAUD);
  gpio_set_function(PIN_I2C_SDA, GPIO_FUNC_I2C);
  gpio_set_function(PIN_I2C_SCL, GPIO_FUNC_I2C);
  gpio_pull_up(PIN_I2C_SDA);
  gpio_pull_up(PIN_I2C_SCL);

  uint8_t who = 0;
  if (!mpu_read(MPU_WHO_AM_I, &who, 1)) {
    return 0;
  }
  /* reset, then clock from the X gyro PLL (more stable than the internal oscillator) */
  if (!mpu_write(MPU_PWR_MGMT_1, 0x80)) return 0;
  sleep_ms(100);
  bool ok = mpu_write(MPU_PWR_MGMT_1, 0x01) &&
            mpu_write(MPU_SMPLRT_DIV, 0x00) &&      /* 1 kHz internal rate with the DLPF on */
            mpu_write(MPU_CONFIG, IMU_DLPF_CFG) &&
            mpu_write(MPU_GYRO_CONFIG, 0x00) &&     /* +-250 dps */
            mpu_write(MPU_ACCEL_CONFIG, 0x00);      /* +-2 g */
  if (!ok) return 0;
  sleep_ms(20);
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
    board_g[i] = (float)g / GYRO_LSB_PER_DPS * DEG_TO_RAD;
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
  if (BATTERY_DIVIDER_RATIO > 0.0f) {
    adc_init();
    adc_gpio_init(PIN_BATTERY_ADC);
  }
}

uint16_t battery_read_mv(void) {
  if (!(BATTERY_DIVIDER_RATIO > 0.0f)) {
    return 0;
  }
  adc_select_input(PIN_BATTERY_ADC - 26);
  uint32_t sum = 0;
  for (int i = 0; i < 8; ++i) {
    sum += adc_read();
  }
  float volts = (float)sum / 8.0f * (3.3f / 4096.0f) * BATTERY_DIVIDER_RATIO;
  return (uint16_t)(volts * 1000.0f + 0.5f);
}

/* ======================= status LED ======================= */

#define LED_PIO pio1
static int led_sm = -1;

void status_led_init(void) {
  int sm = pio_claim_unused_sm(LED_PIO, false);
  if (sm < 0 || !pio_can_add_program(LED_PIO, &ws2812_program)) {
    return;
  }
  uint offset = pio_add_program(LED_PIO, &ws2812_program);
  ws2812_program_init(LED_PIO, (uint)sm, offset, PIN_STATUS_LED, 800000, false);
  led_sm = sm;
}

void status_led_set(uint8_t r, uint8_t g, uint8_t b) {
  if (led_sm < 0 || pio_sm_is_tx_fifo_full(LED_PIO, (uint)led_sm)) {
    return;
  }
  r = (uint8_t)((r * STATUS_LED_BRIGHTNESS) / 255);
  g = (uint8_t)((g * STATUS_LED_BRIGHTNESS) / 255);
  b = (uint8_t)((b * STATUS_LED_BRIGHTNESS) / 255);
#if STATUS_LED_GRB
  uint32_t word = ((uint32_t)g << 16) | ((uint32_t)r << 8) | b;
#else
  uint32_t word = ((uint32_t)r << 16) | ((uint32_t)g << 8) | b;
#endif
  pio_sm_put(LED_PIO, (uint)led_sm, word << 8u);
}
