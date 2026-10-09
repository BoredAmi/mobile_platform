// Command-line probe of the RP2040 link layer (McuLink), without ROS.
//
//   mcu_link_probe <port> [seconds=2] [left_rad_s=0] [right_rad_s=0]
//
// Connects (handshake + default config), sends the wheel velocities at 100 Hz for the given time
// and prints one "key value..." line per result. Used by test_mcu_link.py against fake_board.py;
// on the robot it checks the USB link without starting ros2_control.
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>

#include "jgb_rover_hardware/mcu_link.hpp"

using jgb_rover_hardware::McuLink;

int main(int argc, char ** argv)
{
  if (argc < 2) {
    std::fprintf(stderr, "usage: %s <port> [seconds] [left_rad_s] [right_rad_s]\n", argv[0]);
    return 2;
  }
  McuLink::Options opt;
  opt.port = argv[1];
  opt.fallback_port.clear();
  opt.config = {3960.0f, 0.05f, 0.3f, 0.085f, 0.04f, 0.95f, 250, 4,
    RL_CFG_INVERT_RIGHT_MOTOR | RL_CFG_INVERT_RIGHT_ENCODER | RL_CFG_IDLE_BRAKE};
  const double seconds = argc > 2 ? std::atof(argv[2]) : 2.0;
  const double left = argc > 3 ? std::atof(argv[3]) : 0.0;
  const double right = argc > 4 ? std::atof(argv[4]) : 0.0;

  McuLink link(opt, [](McuLink::Level level, const std::string & msg) {
      static const char * names[] = {"debug", "info", "warn", "error"};
      std::printf("log %s %s\n", names[static_cast<int>(level)], msg.c_str());
      std::fflush(stdout);
    });
  if (!link.connect(3.0)) {
    std::printf("result connect_failed\n");
    return 1;
  }
  std::printf("ready\n");
  std::fflush(stdout);

  const auto start = McuLink::Clock::now();
  auto next = start;
  bool printed_start = false;
  while (McuLink::Clock::now() - start < std::chrono::duration<double>(seconds)) {
    link.poll();
    if (!printed_start && link.state().have_telemetry) {
      std::printf("start_position %.6f %.6f\n", link.state().position[0], link.state().position[1]);
      std::fflush(stdout);
      printed_start = true;
    }
    link.send_velocity(left, right);
    next += std::chrono::milliseconds(10);
    std::this_thread::sleep_until(next);
  }
  link.poll();
  const auto & s = link.state();
  std::printf("position %.6f %.6f\n", s.position[0], s.position[1]);
  std::printf("velocity %.6f %.6f\n", s.velocity[0], s.velocity[1]);
  std::printf("gyro %.6f %.6f %.6f\n", s.gyro[0], s.gyro[1], s.gyro[2]);
  std::printf("accel %.6f %.6f %.6f\n", s.accel[0], s.accel[1], s.accel[2]);
  std::printf("ready_at_end %d\n", link.ready() ? 1 : 0);
  std::printf("resets %u\n", link.resets_seen());
  std::printf("gaps %u\n", link.telemetry_gaps());
  link.disconnect();
  return 0;
}
