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

#include "ddsm_hardware/ddsm_hardware_interface.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"

namespace ddsm_hardware
{

namespace
{

// Bus timing of the USB-RS485 adapter, tuned on the robot
constexpr useconds_t kReplyDelayUs = 1500;     // motor processing time before its reply
constexpr useconds_t kReadRetryDelayUs = 400;  // wait for the rest of a partial reply
constexpr int kReadRetries = 10;
constexpr useconds_t kWriteGapUs = 200;        // gap between speed commands
constexpr useconds_t kSetModeGapUs = 10000;
constexpr int kWarnPeriodMs = 5000;

template<typename T>
std::vector<T> parse_list(const std::string & csv)
{
  std::vector<T> values;
  std::stringstream stream(csv);
  std::string item;
  while (std::getline(stream, item, ',')) {
    values.push_back(static_cast<T>(std::stod(item)));
  }
  return values;
}

std::string param(
  const hardware_interface::HardwareInfo & info, const std::string & name,
  const std::string & default_value = "")
{
  const auto it = info.hardware_parameters.find(name);
  if (it != info.hardware_parameters.end()) {
    return it->second;
  }
  if (default_value.empty()) {
    throw std::runtime_error("missing hardware parameter '" + name + "'");
  }
  return default_value;
}

}  // namespace

DDSMHardwareInterface::~DDSMHardwareInterface()
{
  close_serial_port();
}

hardware_interface::CallbackReturn DDSMHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  std::vector<int> ids, directions;
  std::vector<double> trims;
  try {
    port_ = param(info_, "port");
    rpm_limit_ = std::stoi(param(info_, "rpm_limit", "330"));
    accel_time_ = static_cast<uint8_t>(std::stoi(param(info_, "accel_time", "0")));
    brake_ = static_cast<uint8_t>(std::stoi(param(info_, "brake", "0")));
    ids = parse_list<int>(param(info_, "motor_ids"));
    directions = parse_list<int>(param(info_, "motor_directions"));
    trims = info_.hardware_parameters.count("motor_trims") ?
      parse_list<double>(info_.hardware_parameters.at("motor_trims")) :
      std::vector<double>(ids.size(), 1.0);
  } catch (const std::exception & error) {
    RCLCPP_ERROR(logger_, "invalid hardware parameters: %s", error.what());
    return hardware_interface::CallbackReturn::ERROR;
  }

  const std::size_t joints = info_.joints.size();
  if (ids.size() != joints || directions.size() != joints || trims.size() != joints) {
    RCLCPP_ERROR(
      logger_, "motor_ids, motor_directions and motor_trims need one entry per joint (%zu)",
      joints);
    return hardware_interface::CallbackReturn::ERROR;
  }
  motors_.clear();
  for (std::size_t i = 0; i < joints; ++i) {
    motors_.push_back(Motor{static_cast<uint8_t>(ids[i]), directions[i], trims[i]});
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DDSMHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  if (!open_serial_port()) {
    RCLCPP_ERROR(logger_, "cannot open %s: %s", port_.c_str(), std::strerror(errno));
    close_serial_port();
    return hardware_interface::CallbackReturn::ERROR;
  }
  RCLCPP_INFO(logger_, "opened %s with %zu motors", port_.c_str(), motors_.size());
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DDSMHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  close_serial_port();
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
DDSMHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;
  for (std::size_t i = 0; i < motors_.size(); ++i) {
    interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &motors_[i].position);
    interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &motors_[i].velocity);
  }
  return interfaces;
}

std::vector<hardware_interface::CommandInterface>
DDSMHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (std::size_t i = 0; i < motors_.size(); ++i) {
    interfaces.emplace_back(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &motors_[i].command);
  }
  return interfaces;
}

hardware_interface::CallbackReturn DDSMHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  for (auto & motor : motors_) {
    motor.command = 0.0;
    send(protocol::make_set_mode_frame(motor.id, protocol::kModeVelocity));
    usleep(kSetModeGapUs);
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn DDSMHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  stop_motors();  // otherwise the motors keep their last commanded speed
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type DDSMHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  for (auto & motor : motors_) {
    const auto feedback = query(motor.id);
    if (!feedback) {
      // Keep the last value; move on so one slow motor does not delay the others
      RCLCPP_WARN_THROTTLE(
        logger_, steady_clock_, kWarnPeriodMs, "no feedback from motor %u",
        static_cast<unsigned>(motor.id));
      continue;
    }
    motor.velocity = (feedback->rpm * motor.direction) * (2.0 * M_PI) / 60.0;
    motor.position += motor.velocity * period.seconds();
    if (feedback->error != 0) {
      RCLCPP_WARN_THROTTLE(
        logger_, steady_clock_, kWarnPeriodMs, "motor %u reports %s",
        static_cast<unsigned>(motor.id),
        protocol::error_to_string(feedback->error).c_str());
    }
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type DDSMHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  for (const auto & motor : motors_) {
    int rpm = static_cast<int>((motor.command * 60.0 / (2.0 * M_PI)) * motor.trim);
    rpm = std::clamp(rpm * motor.direction, -rpm_limit_, rpm_limit_);
    send(protocol::make_speed_frame(motor.id, static_cast<int16_t>(rpm), accel_time_, brake_));
    usleep(kWriteGapUs);
  }
  return hardware_interface::return_type::OK;
}

bool DDSMHardwareInterface::open_serial_port()
{
  // Non-blocking, so a missing reply can never stall the control loop
  serial_fd_ = open(port_.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (serial_fd_ < 0) {
    return false;
  }

  termios tty{};
  if (tcgetattr(serial_fd_, &tty) != 0) {
    return false;
  }
  // The DDSM115 bus always runs at 115200 baud
  cfsetospeed(&tty, B115200);
  cfsetispeed(&tty, B115200);

  // Raw 8N1, no flow control
  tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
  tty.c_cflag &= ~(PARENB | PARODD | CSTOPB | CRTSCTS);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_iflag &= ~(IXON | IXOFF | IXANY | IGNBRK | ICRNL);
  tty.c_lflag = 0;
  tty.c_oflag = 0;
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;

  return tcsetattr(serial_fd_, TCSANOW, &tty) == 0;
}

void DDSMHardwareInterface::close_serial_port()
{
  if (serial_fd_ >= 0) {
    close(serial_fd_);
    serial_fd_ = -1;
  }
}

bool DDSMHardwareInterface::send(const protocol::Frame & frame)
{
  return serial_fd_ >= 0 &&
         ::write(serial_fd_, frame.data(), frame.size()) == static_cast<ssize_t>(frame.size());
}

std::optional<protocol::Feedback> DDSMHardwareInterface::query(uint8_t motor_id)
{
  tcflush(serial_fd_, TCIFLUSH);  // drop stale bytes so the reply belongs to this query
  if (!send(protocol::make_query_frame(motor_id))) {
    return std::nullopt;
  }
  usleep(kReplyDelayUs);

  protocol::Frame reply{};
  std::size_t received = 0;
  for (int retry = 0; received < reply.size() && retry < kReadRetries; ) {
    const ssize_t n = ::read(serial_fd_, reply.data() + received, reply.size() - received);
    if (n > 0) {
      received += static_cast<std::size_t>(n);
    } else {
      usleep(kReadRetryDelayUs);
      ++retry;
    }
  }
  if (received != reply.size()) {
    return std::nullopt;
  }
  return protocol::parse_feedback(reply, motor_id);
}

void DDSMHardwareInterface::stop_motors()
{
  for (auto & motor : motors_) {
    motor.command = 0.0;
    send(protocol::make_speed_frame(motor.id, 0, accel_time_, brake_));
    usleep(kWriteGapUs);
  }
}

}  // namespace ddsm_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(ddsm_hardware::DDSMHardwareInterface, hardware_interface::SystemInterface)
