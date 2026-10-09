#include "jgb_rover_hardware/mcu_link.hpp"

#include <cmath>
#include <cstring>
#include <sstream>
#include <thread>
#include <utility>

#include <sys/stat.h>

namespace jgb_rover_hardware
{

namespace
{
constexpr double kTwoPi = 6.283185307179586;
constexpr double kRequestInterval = 0.2;  // s between PING / SET_CONFIG retries

double seconds_since(McuLink::Clock::time_point t)
{
  return std::chrono::duration<double>(McuLink::Clock::now() - t).count();
}

bool exists(const std::string & path)
{
  struct stat st;
  return !path.empty() && ::stat(path.c_str(), &st) == 0;
}

std::string hex(const uint8_t * p, std::size_t n)
{
  static const char * digits = "0123456789abcdef";
  std::string s;
  for (std::size_t i = 0; i < n; ++i) {
    s += digits[p[i] >> 4];
    s += digits[p[i] & 0xF];
  }
  return s;
}
}  // namespace

McuLink::McuLink(Options options, LogFn log)
: opt_(std::move(options)), log_(std::move(log))
{
  rl_decoder_init(&decoder_);
}

bool McuLink::try_open()
{
  last_open_attempt_ = Clock::now();
  std::string device = opt_.port;
  if (!exists(device) && exists(opt_.fallback_port)) {
    device = opt_.fallback_port;
  }
  if (!port_.open(device)) {
    if (!open_error_logged_) {
      log_(Level::Error, "cannot open " + port_.error() +
        (opt_.fallback_port.empty() ? "" : " (fallback " + opt_.fallback_port + " missing too)"));
      open_error_logged_ = true;
    }
    return false;
  }
  open_error_logged_ = false;
  open_port_ = device;
  rl_decoder_init(&decoder_);
  phase_ = Phase::AwaitInfo;
  last_request_ = Clock::time_point{};  // send PING right away
  log_(Level::Info, "opened " + device);
  return true;
}

bool McuLink::connect(double timeout_s)
{
  const auto start = Clock::now();
  if (!port_.is_open()) {
    try_open();
  }
  while (seconds_since(start) < timeout_s) {
    poll();
    if (ready()) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
  std::string where = port_.is_open() ? open_port_ + " is open but the board did not answer "
    "(firmware not flashed, or another program owns the port?)" : "port not available";
  log_(Level::Error, "no handshake with the RP2040 board within " + std::to_string(timeout_s) +
    " s: " + where);
  return false;
}

void McuLink::disconnect()
{
  if (port_.is_open()) {
    send_stop();
    port_.close();
  }
  phase_ = Phase::Closed;
}

void McuLink::send(uint8_t type, const void * payload, std::size_t len)
{
  uint8_t frame[RL_MAX_ENCODED];
  std::size_t n = rl_encode(type, payload, len, frame);
  if (n > 0) {
    port_.write(frame, n);
  }
}

void McuLink::send_velocity(double left, double right)
{
  if (phase_ != Phase::Ready) {
    return;
  }
  rl_cmd_vel_t cmd;
  cmd.vel[0] = std::isfinite(left) ? static_cast<float>(left) : 0.0f;
  cmd.vel[1] = std::isfinite(right) ? static_cast<float>(right) : 0.0f;
  send(RL_MSG_CMD_VEL, &cmd, sizeof(cmd));
}

void McuLink::send_stop()
{
  if (port_.is_open()) {
    send(RL_MSG_STOP, nullptr, 0);
  }
}

bool McuLink::telemetry_fresh() const
{
  return state_.have_telemetry && seconds_since(last_telemetry_) < opt_.telemetry_timeout_s;
}

void McuLink::poll()
{
  if (!port_.is_open()) {
    if (phase_ != Phase::Closed) {
      log_(Level::Error, "lost " + open_port_ + " (" + port_.error() + "), motors stopped by the "
        "board's command timeout; reconnecting");
      phase_ = Phase::Closed;
    }
    if (seconds_since(last_open_attempt_) >= opt_.reconnect_interval_s) {
      try_open();
    }
  }

  uint8_t buf[512];
  long n;
  while ((n = port_.read(buf, sizeof(buf))) > 0) {
    for (long i = 0; i < n; ++i) {
      uint8_t type;
      const uint8_t * payload;
      std::size_t len;
      if (rl_decoder_feed(&decoder_, buf[i], &type, &payload, &len)) {
        handle(type, payload, len);
      }
    }
  }

  if ((phase_ == Phase::AwaitInfo || phase_ == Phase::AwaitConfig) &&
    seconds_since(last_request_) >= kRequestInterval)
  {
    last_request_ = Clock::now();
    if (phase_ == Phase::AwaitInfo) {
      send(RL_MSG_PING, nullptr, 0);
    } else {
      send(RL_MSG_SET_CONFIG, &opt_.config, sizeof(opt_.config));
    }
  }

  if (!telemetry_fresh()) {
    state_.velocity = {0.0, 0.0};
    state_.gyro = {0.0, 0.0, 0.0};
  }
}

void McuLink::handle(uint8_t type, const uint8_t * payload, std::size_t len)
{
  switch (type) {
    case RL_MSG_TELEMETRY:
      if (len == sizeof(rl_telemetry_t)) {
        rl_telemetry_t t;
        std::memcpy(&t, payload, sizeof(t));
        handle_telemetry(t);
      }
      break;
    case RL_MSG_INFO:
      if (len == sizeof(rl_info_t)) {
        std::memcpy(&info_, payload, sizeof(info_));
        if (info_.protocol_version != RL_PROTOCOL_VERSION) {
          log_(Level::Error, "board speaks rover_link protocol " +
            std::to_string(info_.protocol_version) + ", this driver " +
            std::to_string(RL_PROTOCOL_VERSION) + ": flash firmware/rp2040 from this repository");
          break;
        }
        if (phase_ == Phase::AwaitInfo) {
          std::ostringstream s;
          s << "RP2040 board " << hex(info_.board_id, sizeof(info_.board_id)) << ", firmware "
            << (info_.firmware_version >> 8) << "." << (info_.firmware_version & 0xFF) << ", "
            << info_.control_hz << " Hz, IMU WHO_AM_I 0x" << hex(&info_.imu_whoami, 1)
            << (info_.imu_whoami == 0 ? " (NO IMU)" : "")
            << (info_.reset_reason == 1 ? ", last reset by WATCHDOG" : "");
          log_(info_.imu_whoami == 0 ? Level::Warn : Level::Info, s.str());
          phase_ = Phase::AwaitConfig;
          last_request_ = Clock::time_point{};
        }
      }
      break;
    case RL_MSG_CONFIG_ACK:
      if (len == sizeof(rl_config_t) && phase_ == Phase::AwaitConfig) {
        rl_config_t applied;
        std::memcpy(&applied, payload, sizeof(applied));
        if (std::memcmp(&applied, &opt_.config, sizeof(applied)) != 0) {
          log_(Level::Warn, "board clamped some config values to its safe ranges; check "
            "real_hardware.yaml");
        }
        phase_ = Phase::Ready;
        log_(Level::Info, "RP2040 board configured");
      }
      break;
    case RL_MSG_LOG:
      log_(Level::Warn, "[rp2040] " + std::string(reinterpret_cast<const char *>(payload), len));
      break;
    default:
      break;
  }
}

void McuLink::handle_telemetry(const rl_telemetry_t & t)
{
  const bool first = !state_.have_telemetry;
  // seq restarts at 0 when the board resets (watchdog, brown-out, replug)
  const bool reset = !first && t.seq < last_seq_;
  if (reset) {
    ++resets_;
    log_(Level::Warn, "RP2040 board reset detected (seq " + std::to_string(last_seq_) + " -> " +
      std::to_string(t.seq) + "), re-sending config");
    phase_ = Phase::AwaitInfo;
    last_request_ = Clock::time_point{};
  } else if (!first && t.seq != last_seq_ + 1) {
    gaps_ += t.seq - last_seq_ - 1;
  }

  const double rad_per_count = kTwoPi / opt_.config.counts_per_rev;
  for (int i = 0; i < 2; ++i) {
    if (!first && !reset) {
      // wrap-safe difference of 32-bit counters
      total_counts_[i] += static_cast<int32_t>(
        static_cast<uint32_t>(t.enc[i]) - static_cast<uint32_t>(last_counts_[i]));
    }
    last_counts_[i] = t.enc[i];
    state_.position[i] = static_cast<double>(total_counts_[i]) * rad_per_count;
    state_.velocity[i] = t.vel[i];
  }
  const bool imu_ok = t.status & RL_STATUS_IMU_OK;
  for (int i = 0; i < 3; ++i) {
    state_.gyro[i] = imu_ok ? t.gyro[i] : 0.0;
    state_.accel[i] = imu_ok ? t.accel[i] : 0.0;
  }
  log_status_changes(t.status, first);
  last_status_ = t.status;
  last_seq_ = t.seq;
  state_.last = t;
  state_.have_telemetry = true;
  last_telemetry_ = Clock::now();
}

void McuLink::log_status_changes(uint16_t status, bool first)
{
  const uint16_t changed = first ? 0xFFFF : status ^ last_status_;
  if (changed & RL_STATUS_IMU_OK) {
    if (status & RL_STATUS_IMU_OK) {
      log_(Level::Info, "IMU data OK");
    } else {
      log_(Level::Error, "IMU not responding (MPU6050 on GP6/GP7, address 0x68): gyro reported as 0");
    }
  }
  if ((changed & RL_STATUS_CMD_TIMEOUT) && (status & RL_STATUS_CMD_TIMEOUT)) {
    log_(Level::Warn, "board stopped the motors: no velocity command within cmd_timeout_ms");
  }
}

}  // namespace jgb_rover_hardware
