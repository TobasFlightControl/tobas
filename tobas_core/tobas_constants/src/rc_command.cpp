// SPDX-License-Identifier: Apache-2.0
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_constants/rc_command.hpp"

#include <expected>
#include <string>

using namespace std;

namespace tobas
{
namespace
{
constexpr char kRateThrottleText[] = "rate_throttle";
constexpr char kRateThrottleVectorText[] = "rate_throttle_vector";
constexpr char kAngleThrottleText[] = "angle_throttle";
constexpr char kAngleThrottleVectorText[] = "angle_throttle_vector";
constexpr char kAccelYawText[] = "accel_yaw";
constexpr char kAccelPitchYawText[] = "accel_pitch_yaw";
constexpr char kPosVelAccYawText[] = "pos_vel_acc_yaw";
constexpr char kPosVelAccPitchYawText[] = "pos_vel_acc_pitch_yaw";
constexpr char kAccelRateText[] = "accel_rate";
constexpr char kAccelAngleText[] = "accel_angle";
constexpr char kPosVelAccAngleText[] = "pos_vel_acc_angle";
constexpr char kSpeedRollDPitchText[] = "speed_roll_dpitch";
}  // namespace

string rcCommandTextFromEnum(RcCommand cmd)
{
  switch (cmd) {
    case RcCommand::kRateThrottle:
      return kRateThrottleText;
    case RcCommand::kRateThrottleVector:
      return kRateThrottleVectorText;
    case RcCommand::kAngleThrottle:
      return kAngleThrottleText;
    case RcCommand::kAngleThrottleVector:
      return kAngleThrottleVectorText;
    case RcCommand::kAccelYaw:
      return kAccelYawText;
    case RcCommand::kAccelPitchYaw:
      return kAccelPitchYawText;
    case RcCommand::kPosVelAccYaw:
      return kPosVelAccYawText;
    case RcCommand::kPosVelAccPitchYaw:
      return kPosVelAccPitchYawText;
    case RcCommand::kAccelRate:
      return kAccelRateText;
    case RcCommand::kAccelAngle:
      return kAccelAngleText;
    case RcCommand::kPosVelAccAngle:
      return kPosVelAccAngleText;
    case RcCommand::kSpeedRollDPitch:
      return kSpeedRollDPitchText;
    default:
      throw;
  }
}

std::expected<RcCommand, std::string> rcCommandEnumFromText(const std::string& text)
{
  if (text == kRateThrottleText) {
    return RcCommand::kRateThrottle;
  }
  else if (text == kRateThrottleVectorText) {
    return RcCommand::kRateThrottleVector;
  }
  else if (text == kAngleThrottleText) {
    return RcCommand::kAngleThrottle;
  }
  else if (text == kAngleThrottleVectorText) {
    return RcCommand::kAngleThrottleVector;
  }
  else if (text == kAccelYawText) {
    return RcCommand::kAccelYaw;
  }
  else if (text == kAccelPitchYawText) {
    return RcCommand::kAccelPitchYaw;
  }
  else if (text == kPosVelAccYawText) {
    return RcCommand::kPosVelAccYaw;
  }
  else if (text == kPosVelAccPitchYawText) {
    return RcCommand::kPosVelAccPitchYaw;
  }
  else if (text == kAccelRateText) {
    return RcCommand::kAccelRate;
  }
  else if (text == kAccelAngleText) {
    return RcCommand::kAccelAngle;
  }
  else if (text == kPosVelAccAngleText) {
    return RcCommand::kPosVelAccAngle;
  }
  else if (text == kSpeedRollDPitchText) {
    return RcCommand::kSpeedRollDPitch;
  }
  else {
    return std::unexpected("Invalid RC command string.");
  }
}
}  // namespace tobas

namespace YAML
{
Node convert<tobas::RcCommand>::encode(const tobas::RcCommand& rhs)
{
  Node node;
  node = tobas::rcCommandTextFromEnum(rhs);
  return Node(tobas::rcCommandTextFromEnum(rhs));
}

bool convert<tobas::RcCommand>::decode(const Node& node, tobas::RcCommand& rhs)
{
  if (!node.IsScalar()) {
    return false;
  }

  const auto result = tobas::rcCommandEnumFromText(node.as<std::string>());
  if (!result) {
    return false;
  }

  rhs = *result;
  return true;
}
}  // namespace YAML
