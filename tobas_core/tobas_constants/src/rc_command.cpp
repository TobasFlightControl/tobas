// SPDX-License-Identifier: Apache-2.0
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_constants/rc_command.hpp"

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

std::string textFromEnum(RcCommand _cmd)
{
  switch (_cmd) {
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

bool enumFromText(const std::string& _text, RcCommand& _dst)
{
  if (_text == kRateThrottleText) {
    _dst = RcCommand::kRateThrottle;
    return true;
  }
  else if (_text == kRateThrottleVectorText) {
    _dst = RcCommand::kRateThrottleVector;
    return true;
  }
  else if (_text == kAngleThrottleText) {
    _dst = RcCommand::kAngleThrottle;
    return true;
  }
  else if (_text == kAngleThrottleVectorText) {
    _dst = RcCommand::kAngleThrottleVector;
    return true;
  }
  else if (_text == kAccelYawText) {
    _dst = RcCommand::kAccelYaw;
    return true;
  }
  else if (_text == kAccelPitchYawText) {
    _dst = RcCommand::kAccelPitchYaw;
    return true;
  }
  else if (_text == kPosVelAccYawText) {
    _dst = RcCommand::kPosVelAccYaw;
    return true;
  }
  else if (_text == kPosVelAccPitchYawText) {
    _dst = RcCommand::kPosVelAccPitchYaw;
    return true;
  }
  else if (_text == kAccelRateText) {
    _dst = RcCommand::kAccelRate;
    return true;
  }
  else if (_text == kAccelAngleText) {
    _dst = RcCommand::kAccelAngle;
    return true;
  }
  else if (_text == kPosVelAccAngleText) {
    _dst = RcCommand::kPosVelAccAngle;
    return true;
  }
  else if (_text == kSpeedRollDPitchText) {
    _dst = RcCommand::kSpeedRollDPitch;
    return true;
  }
  else {
    return false;
  }
}
}  // namespace tobas

namespace YAML
{
Node convert<tobas::RcCommand>::encode(tobas::RcCommand _rhs)
{
  Node node;
  node = tobas::textFromEnum(_rhs);
  return Node(tobas::textFromEnum(_rhs));
}

bool convert<tobas::RcCommand>::decode(const Node& _node, tobas::RcCommand& _rhs)
{
  if (!_node.IsScalar()) {
    return false;
  }

  return tobas::enumFromText(_node.as<std::string>(), _rhs);
}
}  // namespace YAML
