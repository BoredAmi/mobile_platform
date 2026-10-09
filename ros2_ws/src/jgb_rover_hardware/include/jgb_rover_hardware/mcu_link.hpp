// Host side of the rover_link protocol (firmware/protocol/rover_link.h), independent of ROS so it
// can be tested against a fake board (test/test_mcu_link.py).
//
// Handles the handshake (PING -> INFO, SET_CONFIG -> CONFIG), turns wrapping 32-bit encoder counts
// into continuous wheel angles, detects board resets (re-sends the config, keeps the angles
// continuous) and reconnects after the USB device disappears.
#pragma once

#include <array>
#include <chrono>
#include <cstdint>
#include <functional>
#include <string>

#include "jgb_rover_hardware/serial_port.hpp"
#include "rover_link.h"

namespace jgb_rover_hardware
{

struct McuState
{
  std::array<double, 2> position{};  // rad, [left, right], continuous across wraps / resets
  std::array<double, 2> velocity{};  // rad/s, board estimate (0 when telemetry is stale)
  std::array<double, 3> gyro{};      // rad/s, imu_link (0 when stale or IMU missing)
  std::array<double, 3> accel{};     // m/s^2, imu_link
  rl_telemetry_t last{};             // last raw telemetry
  bool have_telemetry{false};
};

class McuLink
{
public:
  using Clock = std::chrono::steady_clock;
  enum class Level { Debug, Info, Warn, Error };
  using LogFn = std::function<void (Level, const std::string &)>;

  struct Options
  {
    std::string port{"/dev/jgb_rover_mcu"};
    std::string fallback_port{"/dev/ttyACM0"};  // tried when port does not exist ('' = none)
    rl_config_t config{};
    double telemetry_timeout_s{0.1};
    double reconnect_interval_s{1.0};
  };

  McuLink(Options options, LogFn log);

  // Open the port and wait (blocking) until the board answered PING and confirmed the config.
  bool connect(double timeout_s);
  void disconnect();

  // Non-blocking: read and handle everything received; drives the handshake and reconnects.
  void poll();

  // Velocity targets [rad/s]; only sent once the board confirmed the config (with the default
  // config a mirrored motor could turn the wrong way).
  void send_velocity(double left, double right);
  void send_stop();

  bool ready() const {return phase_ == Phase::Ready;}
  bool telemetry_fresh() const;
  const McuState & state() const {return state_;}
  const rl_info_t & info() const {return info_;}
  uint32_t resets_seen() const {return resets_;}
  uint32_t telemetry_gaps() const {return gaps_;}

private:
  enum class Phase { Closed, AwaitInfo, AwaitConfig, Ready };

  bool try_open();
  void send(uint8_t type, const void * payload, std::size_t len);
  void handle(uint8_t type, const uint8_t * payload, std::size_t len);
  void handle_telemetry(const rl_telemetry_t & t);
  void log_status_changes(uint16_t status, bool first);

  Options opt_;
  LogFn log_;
  SerialPort port_;
  std::string open_port_;
  rl_decoder_t decoder_{};
  Phase phase_{Phase::Closed};
  Clock::time_point last_request_{};
  Clock::time_point last_open_attempt_{};
  Clock::time_point last_telemetry_{};
  rl_info_t info_{};
  McuState state_;
  std::array<int64_t, 2> total_counts_{};
  std::array<int32_t, 2> last_counts_{};
  uint32_t last_seq_{0};
  uint16_t last_status_{0};
  uint32_t resets_{0};
  uint32_t gaps_{0};
  bool open_error_logged_{false};
};

}  // namespace jgb_rover_hardware
