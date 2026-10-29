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

#ifndef DDSM_HARDWARE__DDSM_PROTOCOL_HPP_
#define DDSM_HARDWARE__DDSM_PROTOCOL_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

/// Frame encoding of the DDSM115 hub motor RS485 protocol (no I/O, unit tested).
namespace ddsm_hardware::protocol
{

/// Every command and reply on the bus is 10 bytes.
constexpr std::size_t kFrameSize = 10;
using Frame = std::array<uint8_t, kFrameSize>;

constexpr uint8_t kCmdSpeed = 0x64;
constexpr uint8_t kCmdQuery = 0x74;
constexpr uint8_t kCmdSetMode = 0xA0;

constexpr uint8_t kModeCurrent = 0x01;
constexpr uint8_t kModeVelocity = 0x02;
constexpr uint8_t kModePosition = 0x03;

/// Reply of a motor to a query frame (0x74).
struct Feedback
{
  uint8_t motor_id;
  uint8_t mode;
  int16_t current;      ///< raw torque current, -32767..32767 is -8..8 A
  int16_t rpm;          ///< signed speed
  uint8_t temperature;  ///< winding temperature in degrees Celsius
  uint8_t position;     ///< 0..255 over one revolution
  uint8_t error;        ///< fault flags, see error_to_string()
};

/// CRC-8/MAXIM (reflected polynomial 0x8C, initial value 0) of `size` bytes.
uint8_t crc8_maxim(const uint8_t * data, std::size_t size);

/// Selects the control loop of a motor. This frame has no CRC: the last byte is the mode.
Frame make_set_mode_frame(uint8_t motor_id, uint8_t mode = kModeVelocity);

/// Speed command. accel_time 0 uses the default ramp, brake 0xFF brakes.
Frame make_speed_frame(uint8_t motor_id, int16_t rpm, uint8_t accel_time = 0, uint8_t brake = 0);

/// Asks a motor for its feedback.
Frame make_query_frame(uint8_t motor_id);

/// Gives a new ID to the only motor on the bus. The motor takes it after 5 frames, once per
/// power-up, and keeps it when the power is off.
Frame make_set_id_frame(uint8_t new_id);

/// Asks the only motor on the bus for its ID.
Frame make_id_query_frame();

/// Decodes the reply to a query frame. Empty when the CRC is wrong or the reply comes from
/// another motor.
std::optional<Feedback> parse_feedback(const Frame & frame, uint8_t expected_motor_id);

/// Fault flags as text, for example "STALL+OC", or "OK" when none is set.
std::string error_to_string(uint8_t error);

}  // namespace ddsm_hardware::protocol

#endif  // DDSM_HARDWARE__DDSM_PROTOCOL_HPP_
