/*
 * jgb_rover motor / IMU board firmware (Waveshare RP2040-Zero).
 *
 * Every 1 / CONTROL_HZ: sample both encoders and the MPU6050, estimate wheel velocities, run the
 * per-wheel PI + feed-forward speed controllers, drive the motors and send one telemetry frame.
 * Commands arrive over USB CDC (rover_link protocol). Without a command for cmd_timeout_ms the
 * motors are switched off; if the loop itself hangs, the hardware watchdog resets the chip, which
 * also leaves the motor pins off.
 *
 * Status LED: blue blink = waiting for commands, green = driving, yellow = IMU missing,
 * red blink = IMU missing and no host.
 */
#include <string.h>

#include "config.h"
#include "control.h"
#include "hardware/watchdog.h"
#include "hw.h"
#include "pico/bootrom.h"
#include "pico/stdio/driver.h"
#include "pico/stdio_usb.h"
#include "pico/stdlib.h"
#include "pico/unique_id.h"
#include "rover_link.h"
#include "tusb.h"

#define CONTROL_PERIOD_US (1000000 / CONTROL_HZ)

typedef enum { MODE_IDLE, MODE_VEL, MODE_PWM } drive_mode_t;

static rl_config_t cfg = {
    .counts_per_rev = DEFAULT_COUNTS_PER_REV,
    .kp = DEFAULT_KP,
    .ki = DEFAULT_KI,
    .kff = DEFAULT_KFF,
    .deadband_duty = DEFAULT_DEADBAND_DUTY,
    .max_duty = DEFAULT_MAX_DUTY,
    .cmd_timeout_ms = DEFAULT_CMD_TIMEOUT_MS,
    .vel_window = DEFAULT_VEL_WINDOW,
    .flags = DEFAULT_CFG_FLAGS,
};
static bool configured;
static bool rebooted_by_watchdog;

static drive_mode_t mode = MODE_IDLE;
static float cmd[2];               /* rad/s (MODE_VEL) or duty (MODE_PWM) */
static absolute_time_t last_cmd_time;
static bool cmd_seen;
static bool timeout_latched;  /* stopped by the command timeout, until the next command */

static rl_decoder_t decoder;
static uint16_t tx_dropped;

static uint8_t imu_whoami;
static bool imu_ok;
static absolute_time_t imu_retry_time;

/* ------------------------------------------------------------------ link */

static bool link_send(uint8_t type, const void *payload, size_t len) {
  uint8_t frame[RL_MAX_ENCODED];
  size_t n = rl_encode(type, payload, len, frame);
  if (n == 0 || !stdio_usb_connected()) {
    return false;
  }
  /* never block the control loop: drop the frame if the USB buffer cannot take all of it */
  if (tud_cdc_write_available() < n) {
    tx_dropped++;
    return false;
  }
  stdio_usb.out_chars((const char *)frame, (int)n);
  stdio_usb.out_flush();
  return true;
}

static void link_log(const char *text) {
  link_send(RL_MSG_LOG, text, strnlen(text, RL_MAX_PAYLOAD));
}

static void send_info(void) {
  rl_info_t info = {
      .protocol_version = RL_PROTOCOL_VERSION,
      .firmware_version = FIRMWARE_VERSION,
      .control_hz = CONTROL_HZ,
      .imu_whoami = imu_whoami,
      .reset_reason = rebooted_by_watchdog ? 1 : 0,
      .uptime_ms = to_ms_since_boot(get_absolute_time()),
  };
  pico_unique_board_id_t id;
  pico_get_unique_board_id(&id);
  memcpy(info.board_id, id.id, sizeof(info.board_id));
  link_send(RL_MSG_INFO, &info, sizeof(info));
}

static void handle_frame(uint8_t type, const uint8_t *p, size_t len) {
  switch (type) {
    case RL_MSG_CMD_VEL:
    case RL_MSG_CMD_PWM: {
      if (len != 2 * sizeof(float)) break;
      float v[2];
      memcpy(v, p, sizeof(v));
      if (!(v[0] == v[0]) || !(v[1] == v[1])) break;  /* NaN */
      cmd[0] = v[0];
      cmd[1] = v[1];
      mode = type == RL_MSG_CMD_VEL ? MODE_VEL : MODE_PWM;
      last_cmd_time = get_absolute_time();
      cmd_seen = true;
      timeout_latched = false;
      break;
    }
    case RL_MSG_SET_CONFIG:
      if (len != sizeof(rl_config_t)) break;
      memcpy(&cfg, p, sizeof(cfg));
      if (config_sanitize(&cfg)) link_log("config clamped to safe ranges");
      configured = true;
      link_send(RL_MSG_CONFIG_ACK, &cfg, sizeof(cfg));
      break;
    case RL_MSG_PING:
      send_info();
      link_send(RL_MSG_CONFIG, &cfg, sizeof(cfg));
      break;
    case RL_MSG_STOP:
      mode = MODE_IDLE;
      cmd_seen = false;
      break;
    case RL_MSG_BOOTSEL:
      motors_enable(false);
      reset_usb_boot(0, 0);
      break;
    default:
      break;
  }
}

static void link_poll(void) {
  char buf[64];
  int n;
  while ((n = stdio_usb.in_chars(buf, sizeof(buf))) > 0) {
    for (int i = 0; i < n; ++i) {
      uint8_t type;
      const uint8_t *payload;
      size_t len;
      if (rl_decoder_feed(&decoder, (uint8_t)buf[i], &type, &payload, &len)) {
        handle_frame(type, payload, len);
      }
    }
  }
}

/* ------------------------------------------------------------------ control */

static void update_led(uint32_t tick, bool driving) {
  bool blink = (tick / (CONTROL_HZ / 2)) % 2;  /* 1 Hz */
  bool host = stdio_usb_connected();
  if (!imu_ok) {
    if (host) status_led_set(255, 160, 0);
    else status_led_set(blink ? 255 : 0, 0, 0);
  } else if (driving) {
    status_led_set(0, 255, 0);
  } else {
    status_led_set(0, 0, blink ? 255 : 40);
  }
}

int main(void) {
  rebooted_by_watchdog = watchdog_caused_reboot();
  motors_init();  /* first: outputs low before anything else can go wrong */
  encoders_init();
  battery_init();
  status_led_init();
  status_led_set(255, 255, 255);
  stdio_usb_init();
  rl_decoder_init(&decoder);
  config_sanitize(&cfg);

  imu_whoami = imu_init();
  imu_ok = imu_whoami != 0;
  imu_retry_time = make_timeout_time_ms(IMU_RETRY_MS);

  watchdog_enable(WATCHDOG_TIMEOUT_MS, true);

  vel_estimator_t est[2];
  wheel_ctrl_t ctrl[2];
  for (int i = 0; i < 2; ++i) {
    vel_estimator_reset(&est[i]);
    wheel_ctrl_reset(&ctrl[i]);
  }
  /* reported counts accumulate signed deltas, so a direction flag changed by the host only affects
   * motion from then on instead of mirroring the whole count */
  int32_t raw[2], last_raw[2], counts[2] = {0, 0};
  encoders_read(last_raw);
  imu_sample_t imu = {0};
  uint16_t battery_mv = battery_read_mv();
  uint32_t seq = 0;
  bool was_connected = false;

  absolute_time_t next = make_timeout_time_us(CONTROL_PERIOD_US);
  for (uint32_t tick = 0;; ++tick) {
    /* serve the link until the next control instant */
    while (absolute_time_diff_us(get_absolute_time(), next) > 0) {
      link_poll();
      watchdog_update();
    }
    next = delayed_by_us(next, CONTROL_PERIOD_US);
    if (absolute_time_diff_us(get_absolute_time(), next) < 0) {
      next = make_timeout_time_us(CONTROL_PERIOD_US);  /* fell behind: do not try to catch up */
    }

    bool connected = stdio_usb_connected();
    if (connected && !was_connected) {
      send_info();  /* the host may have missed the boot */
    }
    was_connected = connected;

    /* --- sample --- */
    uint32_t t_us = time_us_32();
    encoders_read(raw);
    for (int i = 0; i < 2; ++i) {
      uint32_t delta = (uint32_t)raw[i] - (uint32_t)last_raw[i];  /* wrap-safe */
      bool invert = cfg.flags & (i == 0 ? RL_CFG_INVERT_LEFT_ENCODER : RL_CFG_INVERT_RIGHT_ENCODER);
      counts[i] = (int32_t)((uint32_t)counts[i] + (invert ? (uint32_t)0 - delta : delta));
      last_raw[i] = raw[i];
    }
    float vel[2];
    for (int i = 0; i < 2; ++i) {
      vel[i] = vel_estimator_update(&est[i], counts[i], t_us, cfg.vel_window, cfg.counts_per_rev);
    }

    if (imu_ok) {
      imu_ok = imu_read(&imu);
      if (!imu_ok) {
        link_log("IMU read failed");
        imu_retry_time = make_timeout_time_ms(IMU_RETRY_MS);
      }
    } else if (mode == MODE_IDLE && absolute_time_diff_us(imu_retry_time, get_absolute_time()) >= 0) {
      /* re-init blocks ~120 ms when a device answers, so only while the motors are off */
      imu_whoami = imu_init();
      imu_ok = imu_whoami != 0 && imu_read(&imu);
      imu_retry_time = make_timeout_time_ms(IMU_RETRY_MS);
      if (imu_ok) link_log("IMU recovered");
    }
    if (tick % CONTROL_HZ == 0) {
      battery_mv = battery_read_mv();
    }

    /* --- command watchdog --- */
    bool timed_out = cmd_seen &&
        absolute_time_diff_us(last_cmd_time, get_absolute_time()) > (int64_t)cfg.cmd_timeout_ms * 1000;
    if (timed_out) {
      timeout_latched = true;
      link_log("command timeout: motors off");
    }
    if (timed_out || !connected) {
      mode = MODE_IDLE;
      cmd_seen = false;
    }

    /* --- control --- */
    float target[2] = {0.0f, 0.0f};
    float duty[2] = {0.0f, 0.0f};
    bool sat[2] = {false, false};
    const float dt = 1.0f / CONTROL_HZ;
    if (mode == MODE_VEL) {
      for (int i = 0; i < 2; ++i) {
        target[i] = cmd[i];
        duty[i] = wheel_ctrl_update(&ctrl[i], &cfg, target[i], vel[i], dt, &sat[i]);
      }
    } else {
      for (int i = 0; i < 2; ++i) {
        wheel_ctrl_reset(&ctrl[i]);
        if (mode == MODE_PWM) {
          duty[i] = cmd[i] > cfg.max_duty ? cfg.max_duty : (cmd[i] < -cfg.max_duty ? -cfg.max_duty : cmd[i]);
        }
      }
    }
    bool active = mode != MODE_IDLE;
    if (active) {
      float out[2] = {
          (cfg.flags & RL_CFG_INVERT_LEFT_MOTOR) ? -duty[0] : duty[0],
          (cfg.flags & RL_CFG_INVERT_RIGHT_MOTOR) ? -duty[1] : duty[1],
      };
      motors_enable(true);
      motors_set(out, cfg.flags & RL_CFG_IDLE_BRAKE);
    } else {
      motors_enable(false);
    }

    /* --- telemetry --- */
    rl_telemetry_t tm = {
        .seq = seq++,
        .t_us = t_us,
        .enc = {counts[0], counts[1]},
        .vel = {vel[0], vel[1]},
        .target = {target[0], target[1]},
        .duty = {duty[0], duty[1]},
        .gyro = {imu.gyro[0], imu.gyro[1], imu.gyro[2]},
        .accel = {imu.accel[0], imu.accel[1], imu.accel[2]},
        .imu_temp_c = imu.temp_c,
        .battery_mv = battery_mv,
        .rx_errors = (uint16_t)decoder.errors,
        .tx_dropped = tx_dropped,
    };
    if (!imu_ok) {
      memset(tm.gyro, 0, sizeof(tm.gyro));
      memset(tm.accel, 0, sizeof(tm.accel));
    }
    tm.status = (imu_ok ? RL_STATUS_IMU_OK : 0) |
                (active ? RL_STATUS_MOTORS_ENABLED : 0) |
                (mode == MODE_VEL ? RL_STATUS_MODE_VEL : 0) |
                (mode == MODE_PWM ? RL_STATUS_MODE_PWM : 0) |
                (timeout_latched ? RL_STATUS_CMD_TIMEOUT : 0) |
                (configured ? RL_STATUS_CONFIGURED : 0) |
                (rebooted_by_watchdog ? RL_STATUS_WATCHDOG_REBOOT : 0) |
                (BATTERY_DIVIDER_RATIO > 0.0f ? RL_STATUS_BATTERY_VALID : 0) |
                (sat[0] ? RL_STATUS_SATURATED_L : 0) |
                (sat[1] ? RL_STATUS_SATURATED_R : 0);
    link_send(RL_MSG_TELEMETRY, &tm, sizeof(tm));

    update_led(tick, active);
  }
}
