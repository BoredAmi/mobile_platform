// ros2_control hardware interface for the real jgb_rover: the two drive wheels and the MPU6050,
// both on the RP2040 board (firmware/rp2040), over USB.
//
// Joints (velocity command; position + velocity state): left_wheel_joint, right_wheel_joint.
// Sensor "imu_sensor": orientation.{x,y,z,w} (always identity, unknown), angular_velocity.{x,y,z},
// linear_acceleration.{x,y,z}, read by imu_sensor_broadcaster (-> /imu/data_raw).
#pragma once

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "jgb_rover_hardware/mcu_link.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace jgb_rover_hardware
{

class JgbRoverSystem : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(JgbRoverSystem)

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State & previous) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  rclcpp::Logger logger_{rclcpp::get_logger("JgbRoverSystem")};
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};  // for throttled warnings
  std::unique_ptr<McuLink> link_;
  McuLink::Options options_;
  double handshake_timeout_s_{3.0};

  std::array<std::string, 2> joint_names_;  // [left, right]
  std::array<double, 2> position_{};
  std::array<double, 2> velocity_{};
  std::array<double, 2> command_{};

  std::string imu_name_;
  std::array<double, 4> orientation_{0.0, 0.0, 0.0, 1.0};
  std::array<double, 3> angular_velocity_{};
  std::array<double, 3> linear_acceleration_{};

  bool active_{false};
};

}  // namespace jgb_rover_hardware
