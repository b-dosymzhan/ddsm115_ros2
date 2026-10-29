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

#include "ddsm_hardware/ddsm_protocol.hpp"

#include <string>

namespace ddsm_hardware::protocol
{

namespace
{

/// Fills byte 9 with the CRC of bytes 0..8.
Frame with_crc(Frame frame)
{
  frame[kFrameSize - 1] = crc8_maxim(frame.data(), kFrameSize - 1);
  return frame;
}

int16_t to_int16(uint8_t high, uint8_t low)
{
  return static_cast<int16_t>(static_cast<uint16_t>((high << 8) | low));
}

}  // namespace

uint8_t crc8_maxim(const uint8_t * data, std::size_t size)
{
  uint8_t crc = 0x00;
  for (std::size_t i = 0; i < size; ++i) {
    crc ^= data[i];
    for (int bit = 0; bit < 8; ++bit) {
      crc = (crc & 0x01) ? static_cast<uint8_t>((crc >> 1) ^ 0x8C) : static_cast<uint8_t>(crc >> 1);
    }
  }
  return crc;
}

Frame make_set_mode_frame(uint8_t motor_id, uint8_t mode)
{
  return Frame{motor_id, kCmdSetMode, 0, 0, 0, 0, 0, 0, 0, mode};
}

Frame make_speed_frame(uint8_t motor_id, int16_t rpm, uint8_t accel_time, uint8_t brake)
{
  const auto value = static_cast<uint16_t>(rpm);  // two's complement on the wire
  return with_crc(
    Frame{
      motor_id, kCmdSpeed, static_cast<uint8_t>(value >> 8), static_cast<uint8_t>(value & 0xFF),
      0, 0, accel_time, brake, 0, 0});
}

Frame make_query_frame(uint8_t motor_id)
{
  return with_crc(Frame{motor_id, kCmdQuery, 0, 0, 0, 0, 0, 0, 0, 0});
}

Frame make_set_id_frame(uint8_t new_id)
{
  return with_crc(Frame{0xAA, 0x55, 0x53, new_id, 0, 0, 0, 0, 0, 0});
}

Frame make_id_query_frame()
{
  return with_crc(Frame{0xC8, 0x64, 0, 0, 0, 0, 0, 0, 0, 0});
}

std::optional<Feedback> parse_feedback(const Frame & frame, uint8_t expected_motor_id)
{
  if (frame[0] != expected_motor_id ||
    crc8_maxim(frame.data(), kFrameSize - 1) != frame[kFrameSize - 1])
  {
    return std::nullopt;
  }
  return Feedback{
    frame[0],
    frame[1],
    to_int16(frame[2], frame[3]),
    to_int16(frame[4], frame[5]),
    frame[6],
    frame[7],
    frame[8]};
}

std::string error_to_string(uint8_t error)
{
  static constexpr std::pair<uint8_t, const char *> kFlags[] = {
    {1 << 3, "STALL"}, {1 << 2, "PHASE_OVERCURRENT"}, {1 << 1, "OVERCURRENT"}, {1 << 0, "SENSOR"}};
  std::string text;
  for (const auto & [mask, name] : kFlags) {
    if (error & mask) {
      text += text.empty() ? name : std::string("+") + name;
    }
  }
  return text.empty() ? "OK" : text;
}

}  // namespace ddsm_hardware::protocol
