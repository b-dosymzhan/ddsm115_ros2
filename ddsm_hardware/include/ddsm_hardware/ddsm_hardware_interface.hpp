// Copyright 2026 Dosymzhan Birzhanuly
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef DDSM_HARDWARE__DDSM_HARDWARE_INTERFACE_HPP_
#define DDSM_HARDWARE__DDSM_HARDWARE_INTERFACE_HPP_

#include <optional>
#include <string>
#include <vector>

#include "ddsm_hardware/ddsm_protocol.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace ddsm_hardware
{

/**
 * ros2_control system interface for DDSM115 hub motors sharing one RS485 bus (115200 baud).
 *
 * Each joint is one motor with a velocity command and velocity + position states. The joint
 * order of the <ros2_control> tag maps onto the motor lists below.
 *
 * Hardware parameters:
 *   port              serial device of the USB-RS485 adapter
 *   motor_ids         comma separated motor IDs, one per joint
 *   motor_directions  1 or -1 per motor, so that a positive joint velocity drives forward
 *   motor_trims       optional speed factor per motor (default 1.0) to make the robot drive
 *                     straight
 *   rpm_limit         command clamp (the motors accept up to 330 rpm)
 *   accel_time        optional ramp time sent with every speed command (0 = motor default)
 *   brake             optional brake byte sent with every speed command (0xFF = brake)
 */
class DDSMHardwareInterface : public hardware_interface::SystemInterface
{
public:
  ~DDSMHardwareInterface() override;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct Motor
  {
    uint8_t id;
    int direction;          ///< 1 or -1
    double trim;            ///< speed factor applied to commands
    double position = 0.0;  ///< rad, integrated from the reported speed
    double velocity = 0.0;  ///< rad/s
    double command = 0.0;   ///< rad/s
  };

  bool open_serial_port();
  void close_serial_port();
  bool send(const protocol::Frame & frame);
  /// Sends a query and waits for the reply (about 2 ms per motor).
  std::optional<protocol::Feedback> query(uint8_t motor_id);
  void stop_motors();

  rclcpp::Logger logger_ = rclcpp::get_logger("DDSMHardwareInterface");
  rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

  std::string port_;
  int serial_fd_ = -1;
  int rpm_limit_ = 330;
  uint8_t accel_time_ = 0;
  uint8_t brake_ = 0;
  std::vector<Motor> motors_;  ///< same order as the joints
};

}  // namespace ddsm_hardware

#endif  // DDSM_HARDWARE__DDSM_HARDWARE_INTERFACE_HPP_
