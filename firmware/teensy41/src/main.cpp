/*
 * jgb_rover motor / IMU board firmware (Teensy 4.1).
 *
 * Every 1 / CONTROL_HZ: sample both encoders and the MPU6050, estimate wheel velocities, run the
 * per-wheel PI + feed-forward speed controllers, drive the motors and send one telemetry frame.
 * Commands arrive over USB serial (rover_link protocol). Without a command for cmd_timeout_ms the
 * motors are switched off. If the loop itself stalls, a timer interrupt switches the driver off
 * after LOOP_STALL_MS and the hardware watchdog resets the chip.
 *
 * Status LED (pin 13): on = driving, 1 Hz blink = waiting for commands, short flash every 2 s =
 * no host, fast 5 Hz blink = IMU missing.
 */
#include <Arduino.h>
#include <string.h>

#include "config.h"
#include "control.h"
#include "hw.h"
#include "rover_link.h"

#define CONTROL_PERIOD_US (1000000UL / CONTROL_HZ)

typedef enum { MODE_IDLE, MODE_VEL, MODE_PWM } drive_mode_t;

static rl_config_t cfg = {
    DEFAULT_COUNTS_PER_REV, DEFAULT_KP, DEFAULT_KI, DEFAULT_KFF, DEFAULT_DEADBAND_DUTY,
    DEFAULT_MAX_DUTY, DEFAULT_CMD_TIMEOUT_MS, DEFAULT_VEL_WINDOW, DEFAULT_CFG_FLAGS,
};
static bool configured;
static bool rebooted_by_watchdog;

static drive_mode_t mode = MODE_IDLE;
static float cmd[2];               /* rad/s (MODE_VEL) or duty (MODE_PWM) */
static uint32_t last_cmd_ms;
static bool cmd_seen;
static bool timeout_latched;       /* stopped by the command timeout, until the next command */

static rl_decoder_t decoder;
static uint16_t tx_dropped;

static uint8_t imu_whoami;
static bool imu_ok;
static uint32_t imu_retry_ms;

/* ------------------------------------------------------------------ link */

static bool host_connected(void) {
  return (bool)Serial;  /* USB configured and DTR raised: a program has the port open */
}

static bool link_send(uint8_t type, const void *payload, size_t len) {
  uint8_t frame[RL_MAX_ENCODED];
  size_t n = rl_encode(type, payload, len, frame);
  if (n == 0 || !host_connected()) {
    return false;
  }
  /* never block the control loop: drop the frame if the USB buffer cannot take all of it */
  if ((size_t)Serial.availableForWrite() < n) {
    tx_dropped++;
    return false;
  }
  Serial.write(frame, n);
  Serial.send_now();
  return true;
}

static void link_log(const char *text) {
  link_send(RL_MSG_LOG, text, strnlen(text, RL_MAX_PAYLOAD));
}

static void send_info(void) {
  rl_info_t info;
  memset(&info, 0, sizeof(info));
  info.protocol_version = RL_PROTOCOL_VERSION;
  info.firmware_version = FIRMWARE_VERSION;
  info.control_hz = CONTROL_HZ;
  info.imu_whoami = imu_whoami;
  info.reset_reason = rebooted_by_watchdog ? 1 : 0;
  info.uptime_ms = millis();
  board_unique_id(info.board_id);
  link_send(RL_MSG_INFO, &info, sizeof(info));
}

static void handle_frame(uint8_t type, const uint8_t *p, size_t len) {
  switch (type) {
    case RL_MSG_CMD_VEL:
    case RL_MSG_CMD_PWM: {
      if (len != 2 * sizeof(float)) break;
      float v[2];
      memcpy(v, p, sizeof(v));
      if (isnan(v[0]) || isnan(v[1])) break;
      cmd[0] = v[0];
      cmd[1] = v[1];
      mode = type == RL_MSG_CMD_VEL ? MODE_VEL : MODE_PWM;
      last_cmd_ms = millis();
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
    case RL_MSG_BOOTLOADER:
      reboot_to_bootloader();  /* does not return */
      break;
    default:
      break;
  }
}

static void link_poll(void) {
  uint8_t buf[64];
  int n;
  while ((n = Serial.available()) > 0) {
    if (n > (int)sizeof(buf)) n = sizeof(buf);
    n = Serial.readBytes((char *)buf, n);
    for (int i = 0; i < n; ++i) {
      uint8_t type;
      const uint8_t *payload;
      size_t len;
      if (rl_decoder_feed(&decoder, buf[i], &type, &payload, &len)) {
        handle_frame(type, payload, len);
      }
    }
  }
}

/* ------------------------------------------------------------------ control */

static void update_led(uint32_t tick, bool driving, bool host) {
  const uint32_t ms = tick * (1000 / CONTROL_HZ);
  bool on;
  if (!imu_ok) on = (ms / 100) % 2;             /* 5 Hz */
  else if (driving) on = true;
  else if (host) on = (ms / 500) % 2;           /* 1 Hz */
  else on = (ms % 2000) < 100;                  /* short flash every 2 s */
  status_led(on);
}

static vel_estimator_t est[2];
static wheel_ctrl_t ctrl[2];
static imu_sample_t imu;
static uint16_t battery_mv;
static uint32_t seq;
static bool was_connected;
static int32_t last_raw[2], counts[2];
static uint32_t next_us;
static uint32_t tick;

void setup() {
  rebooted_by_watchdog = reset_was_watchdog();
  motors_init();  /* first: outputs low before anything else can go wrong */
  status_led_init();
  status_led(true);
  encoders_init();
  battery_init();
  Serial.begin(115200);  /* USB: the rate is ignored */
  Serial.setTimeout(0);
  rl_decoder_init(&decoder);
  config_sanitize(&cfg);

  imu_whoami = imu_init();
  imu_ok = imu_whoami != 0;
  imu_retry_ms = millis() + IMU_RETRY_MS;

  for (int i = 0; i < 2; ++i) {
    vel_estimator_reset(&est[i]);
    wheel_ctrl_reset(&ctrl[i]);
  }
  /* reported counts accumulate signed deltas, so a direction flag changed by the host only affects
   * motion from then on instead of mirroring the whole count */
  encoders_read(last_raw);
  battery_mv = battery_read_mv();

  watchdog_start();
  loop_guard_start();
  next_us = micros() + CONTROL_PERIOD_US;
}

void loop() {
  /* serve the link until the next control instant */
  while ((int32_t)(micros() - next_us) < 0) {
    link_poll();
    watchdog_feed();
    loop_alive();
  }
  next_us += CONTROL_PERIOD_US;
  if ((int32_t)(micros() - next_us) > 0) {
    next_us = micros() + CONTROL_PERIOD_US;  /* fell behind: do not try to catch up */
  }
  ++tick;

  bool connected = host_connected();
  if (connected && !was_connected) {
    send_info();  /* the host may have missed the boot */
  }
  was_connected = connected;

  /* --- sample --- */
  uint32_t t_us = micros();
  int32_t raw[2];
  encoders_read(raw);
  float vel[2];
  for (int i = 0; i < 2; ++i) {
    uint32_t delta = (uint32_t)raw[i] - (uint32_t)last_raw[i];  /* wrap-safe */
    bool invert = cfg.flags & (i == 0 ? RL_CFG_INVERT_LEFT_ENCODER : RL_CFG_INVERT_RIGHT_ENCODER);
    counts[i] = (int32_t)((uint32_t)counts[i] + (invert ? (uint32_t)0 - delta : delta));
    last_raw[i] = raw[i];
    vel[i] = vel_estimator_update(&est[i], counts[i], t_us, cfg.vel_window, cfg.counts_per_rev);
  }

  if (imu_ok) {
    imu_ok = imu_read(&imu);
    if (!imu_ok) {
      link_log("IMU read failed");
      imu_retry_ms = millis() + IMU_RETRY_MS;
    }
  } else if (mode == MODE_IDLE && (int32_t)(millis() - imu_retry_ms) >= 0) {
    /* re-init blocks ~120 ms when a device answers, so only while the motors are off */
    imu_whoami = imu_init();
    imu_ok = imu_whoami != 0 && imu_read(&imu);
    imu_retry_ms = millis() + IMU_RETRY_MS;
    if (imu_ok) link_log("IMU recovered");
  }
  if (tick % CONTROL_HZ == 0) {
    battery_mv = battery_read_mv();
  }

  /* --- command watchdog --- */
  bool timed_out = cmd_seen && (millis() - last_cmd_ms) > cfg.cmd_timeout_ms;
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
        duty[i] = constrain(cmd[i], -cfg.max_duty, cfg.max_duty);
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
  rl_telemetry_t tm;
  memset(&tm, 0, sizeof(tm));
  tm.seq = seq++;
  tm.t_us = t_us;
  for (int i = 0; i < 2; ++i) {
    tm.enc[i] = counts[i];
    tm.vel[i] = vel[i];
    tm.target[i] = target[i];
    tm.duty[i] = duty[i];
  }
  if (imu_ok) {
    for (int i = 0; i < 3; ++i) {
      tm.gyro[i] = imu.gyro[i];
      tm.accel[i] = imu.accel[i];
    }
  }
  tm.imu_temp_c = imu.temp_c;
  tm.battery_mv = battery_mv;
  tm.rx_errors = (uint16_t)decoder.errors;
  tm.tx_dropped = tx_dropped;
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

  update_led(tick, active, connected);
  watchdog_feed();
  loop_alive();
}
