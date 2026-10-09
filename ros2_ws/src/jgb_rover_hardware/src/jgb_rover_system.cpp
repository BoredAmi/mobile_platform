#include "jgb_rover_hardware/jgb_rover_system.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <stdexcept>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace jgb_rover_hardware
{

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;

namespace
{
std::string param(const hardware_interface::HardwareInfo & info, const std::string & name,
  const std::string & fallback)
{
  auto it = info.hardware_parameters.find(name);
  return it == info.hardware_parameters.end() || it->second.empty() ? fallback : it->second;
}

double param_double(const hardware_interface::HardwareInfo & info, const std::string & name,
  double fallback)
{
  auto it = info.hardware_parameters.find(name);
  if (it == info.hardware_parameters.end() || it->second.empty()) {
    return fallback;
  }
  try {
    return std::stod(it->second);
  } catch (const std::exception &) {
    throw std::invalid_argument("hardware parameter '" + name + "' is not a number: " + it->second);
  }
}

bool param_bool(const hardware_interface::HardwareInfo & info, const std::string & name, bool fallback)
{
  std::string v = param(info, name, fallback ? "true" : "false");
  std::transform(v.begin(), v.end(), v.begin(), [](unsigned char c) {return std::tolower(c);});
  if (v == "true" || v == "1" || v == "yes") {
    return true;
  }
  if (v == "false" || v == "0" || v == "no") {
    return false;
  }
  throw std::invalid_argument("hardware parameter '" + name + "' is not a boolean: " + v);
}
}  // namespace

CallbackReturn JgbRoverSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) {
    return CallbackReturn::ERROR;
  }

  // ---- joints: exactly the two wheels, velocity command, position + velocity state ----
  const std::string left = param(info_, "left_wheel_joint", "left_wheel_joint");
  const std::string right = param(info_, "right_wheel_joint", "right_wheel_joint");
  if (info_.joints.size() != 2) {
    RCLCPP_ERROR(logger_, "expected 2 joints (%s, %s), got %zu", left.c_str(), right.c_str(),
      info_.joints.size());
    return CallbackReturn::ERROR;
  }
  for (const auto & j : info_.joints) {
    if (j.name != left && j.name != right) {
      RCLCPP_ERROR(logger_, "unexpected joint '%s'", j.name.c_str());
      return CallbackReturn::ERROR;
    }
    if (j.command_interfaces.size() != 1 ||
      j.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_ERROR(logger_, "joint '%s' needs exactly one 'velocity' command interface", j.name.c_str());
      return CallbackReturn::ERROR;
    }
  }
  joint_names_ = {left, right};

  // ---- optional IMU sensor ----
  if (info_.sensors.size() > 1) {
    RCLCPP_ERROR(logger_, "at most one sensor (the IMU) is supported");
    return CallbackReturn::ERROR;
  }
  if (!info_.sensors.empty()) {
    imu_name_ = info_.sensors[0].name;
  }

  // ---- board configuration (sent at every connect) ----
  try {
    options_.port = param(info_, "serial_port", "/dev/jgb_rover_mcu");
    options_.fallback_port = param(info_, "fallback_serial_port", "/dev/ttyACM0");
    if (options_.fallback_port == "none") {
      options_.fallback_port.clear();
    }
    options_.telemetry_timeout_s = param_double(info_, "telemetry_timeout_s", 0.1);
    handshake_timeout_s_ = param_double(info_, "handshake_timeout_s", 3.0);

    rl_config_t & c = options_.config;
    c.counts_per_rev = static_cast<float>(param_double(info_, "counts_per_rev", 3960.0));
    c.kp = static_cast<float>(param_double(info_, "kp", 0.05));
    c.ki = static_cast<float>(param_double(info_, "ki", 0.3));
    c.kff = static_cast<float>(param_double(info_, "kff", 0.085));
    c.deadband_duty = static_cast<float>(param_double(info_, "deadband_duty", 0.04));
    c.max_duty = static_cast<float>(param_double(info_, "max_duty", 0.95));
    c.cmd_timeout_ms = static_cast<uint16_t>(param_double(info_, "cmd_timeout_ms", 250));
    c.vel_window = static_cast<uint8_t>(param_double(info_, "vel_window", 4));
    c.flags = static_cast<uint8_t>(
      (param_bool(info_, "invert_left_motor", false) ? RL_CFG_INVERT_LEFT_MOTOR : 0) |
      (param_bool(info_, "invert_right_motor", true) ? RL_CFG_INVERT_RIGHT_MOTOR : 0) |
      (param_bool(info_, "invert_left_encoder", false) ? RL_CFG_INVERT_LEFT_ENCODER : 0) |
      (param_bool(info_, "invert_right_encoder", true) ? RL_CFG_INVERT_RIGHT_ENCODER : 0) |
      (param_bool(info_, "idle_brake", true) ? RL_CFG_IDLE_BRAKE : 0));
  } catch (const std::invalid_argument & e) {
    RCLCPP_ERROR(logger_, "%s", e.what());
    return CallbackReturn::ERROR;
  }
  if (!(options_.config.counts_per_rev > 0.0f)) {
    RCLCPP_ERROR(logger_, "counts_per_rev must be > 0");
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> JgbRoverSystem::export_state_interfaces()
{
  namespace hi = hardware_interface;
  std::vector<hi::StateInterface> s;
  for (int i = 0; i < 2; ++i) {
    s.emplace_back(joint_names_[i], hi::HW_IF_POSITION, &position_[i]);
    s.emplace_back(joint_names_[i], hi::HW_IF_VELOCITY, &velocity_[i]);
  }
  if (!imu_name_.empty()) {
    const char * q[] = {"orientation.x", "orientation.y", "orientation.z", "orientation.w"};
    const char * xyz[] = {"x", "y", "z"};
    for (int i = 0; i < 4; ++i) {
      s.emplace_back(imu_name_, q[i], &orientation_[i]);
    }
    for (int i = 0; i < 3; ++i) {
      s.emplace_back(imu_name_, std::string("angular_velocity.") + xyz[i], &angular_velocity_[i]);
    }
    for (int i = 0; i < 3; ++i) {
      s.emplace_back(imu_name_, std::string("linear_acceleration.") + xyz[i], &linear_acceleration_[i]);
    }
  }
  return s;
}

std::vector<hardware_interface::CommandInterface> JgbRoverSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> c;
  for (int i = 0; i < 2; ++i) {
    c.emplace_back(joint_names_[i], hardware_interface::HW_IF_VELOCITY, &command_[i]);
  }
  return c;
}

CallbackReturn JgbRoverSystem::on_configure(const rclcpp_lifecycle::State &)
{
  link_ = std::make_unique<McuLink>(options_, [this](McuLink::Level level, const std::string & msg) {
        switch (level) {
          case McuLink::Level::Debug: RCLCPP_DEBUG(logger_, "%s", msg.c_str()); break;
          case McuLink::Level::Info: RCLCPP_INFO(logger_, "%s", msg.c_str()); break;
          case McuLink::Level::Warn: RCLCPP_WARN(logger_, "%s", msg.c_str()); break;
          case McuLink::Level::Error: RCLCPP_ERROR(logger_, "%s", msg.c_str()); break;
        }
      });
  RCLCPP_INFO(logger_, "connecting to the RP2040 board on %s ...", options_.port.c_str());
  if (!link_->connect(handshake_timeout_s_)) {
    link_.reset();
    return CallbackReturn::ERROR;
  }
  // start from the board's current encoder state
  const auto deadline = McuLink::Clock::now() + std::chrono::milliseconds(200);
  while (!link_->state().have_telemetry && McuLink::Clock::now() < deadline) {
    link_->poll();
  }
  position_ = link_->state().position;
  velocity_ = {0.0, 0.0};
  command_ = {0.0, 0.0};
  return CallbackReturn::SUCCESS;
}

CallbackReturn JgbRoverSystem::on_cleanup(const rclcpp_lifecycle::State &)
{
  if (link_) {
    link_->disconnect();
    link_.reset();
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn JgbRoverSystem::on_activate(const rclcpp_lifecycle::State &)
{
  command_ = {0.0, 0.0};
  active_ = true;
  return CallbackReturn::SUCCESS;
}

CallbackReturn JgbRoverSystem::on_deactivate(const rclcpp_lifecycle::State &)
{
  active_ = false;
  if (link_) {
    link_->send_stop();
  }
  return CallbackReturn::SUCCESS;
}

return_type JgbRoverSystem::read(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!link_) {
    return return_type::ERROR;
  }
  link_->poll();
  const McuState & st = link_->state();
  if (!link_->telemetry_fresh()) {
    RCLCPP_WARN_THROTTLE(logger_, steady_clock_, 2000,
      "no telemetry from the RP2040 board: wheel velocities and gyro reported as 0");
  }
  position_ = st.position;
  velocity_ = st.velocity;
  angular_velocity_ = st.gyro;
  linear_acceleration_ = st.accel;
  return return_type::OK;
}

return_type JgbRoverSystem::write(const rclcpp::Time &, const rclcpp::Duration &)
{
  if (!link_) {
    return return_type::ERROR;
  }
  if (active_) {
    link_->send_velocity(command_[0], command_[1]);
  }
  return return_type::OK;
}

}  // namespace jgb_rover_hardware

PLUGINLIB_EXPORT_CLASS(jgb_rover_hardware::JgbRoverSystem, hardware_interface::SystemInterface)
