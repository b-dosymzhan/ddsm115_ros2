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

#include <gtest/gtest.h>

#include <cstring>

#include "ddsm_hardware/ddsm_protocol.hpp"

using ddsm_hardware::protocol::Frame;
namespace protocol = ddsm_hardware::protocol;

TEST(Crc8Maxim, MatchesStandardCheckValue)
{
  // Published check value of CRC-8/MAXIM for the ASCII string "123456789"
  const char * check = "123456789";
  EXPECT_EQ(
    protocol::crc8_maxim(reinterpret_cast<const uint8_t *>(check), std::strlen(check)),
    0xA1);
}

// The expected frames below are the examples of the Waveshare DDSM115 wiki

TEST(Frames, SpeedCommand)
{
  EXPECT_EQ(
    protocol::make_speed_frame(1, 100),
    (Frame{0x01, 0x64, 0x00, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x4F}));
  EXPECT_EQ(
    protocol::make_speed_frame(1, 50),
    (Frame{0x01, 0x64, 0x00, 0x32, 0x00, 0x00, 0x00, 0x00, 0x00, 0xD3}));
}

TEST(Frames, Brake)
{
  EXPECT_EQ(
    protocol::make_speed_frame(1, 0, 0, 0xFF),
    (Frame{0x01, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF, 0x00, 0xD1}));
}

TEST(Frames, NegativeSpeedIsTwosComplement)
{
  EXPECT_EQ(
    protocol::make_speed_frame(3, -100),
    (Frame{0x03, 0x64, 0xFF, 0x9C, 0x00, 0x00, 0x00, 0x00, 0x00, 0xCB}));
}

TEST(Frames, SpeedCommandCarriesAccelerationAndBrake)
{
  EXPECT_EQ(
    protocol::make_speed_frame(2, 330, 5, 0xFF),
    (Frame{0x02, 0x64, 0x01, 0x4A, 0x00, 0x00, 0x05, 0xFF, 0x00, 0x96}));
}

TEST(Frames, Query)
{
  EXPECT_EQ(
    protocol::make_query_frame(4),
    (Frame{0x04, 0x74, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02}));
}

TEST(Frames, SetModeHasNoCrc)
{
  EXPECT_EQ(
    protocol::make_set_mode_frame(1),
    (Frame{0x01, 0xA0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02}));
  EXPECT_EQ(
    protocol::make_set_mode_frame(2, protocol::kModePosition),
    (Frame{0x02, 0xA0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x03}));
}

TEST(Frames, SetId)
{
  EXPECT_EQ(
    protocol::make_set_id_frame(1),
    (Frame{0xAA, 0x55, 0x53, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0xCB}));
  EXPECT_EQ(
    protocol::make_set_id_frame(4),
    (Frame{0xAA, 0x55, 0x53, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x20}));
}

TEST(Frames, IdQuery)
{
  EXPECT_EQ(
    protocol::make_id_query_frame(),
    (Frame{0xC8, 0x64, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xDE}));
}

// Reply of motor 2: velocity mode, current 5, -100 rpm, 30 degC, position 128 (180 deg), stall
constexpr Frame kReply{0x02, 0x02, 0x00, 0x05, 0xFF, 0x9C, 0x1E, 0x80, 0x08, 0x55};

TEST(Feedback, DecodesValidReply)
{
  const auto feedback = protocol::parse_feedback(kReply, 2);
  ASSERT_TRUE(feedback.has_value());
  EXPECT_EQ(feedback->motor_id, 2);
  EXPECT_EQ(feedback->mode, 0x02);
  EXPECT_EQ(feedback->current, 5);
  EXPECT_EQ(feedback->rpm, -100);
  EXPECT_EQ(feedback->temperature, 30);
  EXPECT_EQ(feedback->position, 128);
  EXPECT_EQ(protocol::error_to_string(feedback->error), "STALL");
}

TEST(Feedback, RejectsBadCrc)
{
  Frame reply = kReply;
  reply[5] ^= 0x01;  // corrupt one bit
  EXPECT_FALSE(protocol::parse_feedback(reply, 2).has_value());
}

TEST(Feedback, RejectsReplyFromAnotherMotor)
{
  EXPECT_FALSE(protocol::parse_feedback(kReply, 3).has_value());
}

TEST(Feedback, ErrorFlagsToString)
{
  EXPECT_EQ(protocol::error_to_string(0x00), "OK");
  EXPECT_EQ(protocol::error_to_string(0x0A), "STALL+OVERCURRENT");
  EXPECT_EQ(protocol::error_to_string(0x0F), "STALL+PHASE_OVERCURRENT+OVERCURRENT+SENSOR");
}
