// SPDX-License-Identifier: Apache-2.0
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_constants/rc_command.hpp"

#include <iostream>

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

string textFromEnum(RcCommand cmd)
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

bool enumFromText(const string& text, RcCommand& dst)
{
  if (text == kRateThrottleText) {
    dst = RcCommand::kRateThrottle;
    return true;
  }
  else if (text == kRateThrottleVectorText) {
    dst = RcCommand::kRateThrottleVector;
    return true;
  }
  else if (text == kAngleThrottleText) {
    dst = RcCommand::kAngleThrottle;
    return true;
  }
  else if (text == kAngleThrottleVectorText) {
    dst = RcCommand::kAngleThrottleVector;
    return true;
  }
  else if (text == kAccelYawText) {
    dst = RcCommand::kAccelYaw;
    return true;
  }
  else if (text == kAccelPitchYawText) {
    dst = RcCommand::kAccelPitchYaw;
    return true;
  }
  else if (text == kPosVelAccYawText) {
    dst = RcCommand::kPosVelAccYaw;
    return true;
  }
  else if (text == kPosVelAccPitchYawText) {
    dst = RcCommand::kPosVelAccPitchYaw;
    return true;
  }
  else if (text == kAccelRateText) {
    dst = RcCommand::kAccelRate;
    return true;
  }
  else if (text == kAccelAngleText) {
    dst = RcCommand::kAccelAngle;
    return true;
  }
  else if (text == kPosVelAccAngleText) {
    dst = RcCommand::kPosVelAccAngle;
    return true;
  }
  else if (text == kSpeedRollDPitchText) {
    dst = RcCommand::kSpeedRollDPitch;
    return true;
  }
  else {
    cerr << "Invalid RC command: " << text << endl;
    return false;
  }
}
}  // namespace tobas

namespace YAML
{
Node convert<tobas::RcCommand>::encode(const tobas::RcCommand& rhs)
{
  Node node;
  node = tobas::textFromEnum(rhs);
  return Node(tobas::textFromEnum(rhs));
}

bool convert<tobas::RcCommand>::decode(const Node& node, tobas::RcCommand& rhs)
{
  if (!node.IsScalar()) {
    return false;
  }

  return tobas::enumFromText(node.as<string>(), rhs);
}
}  // namespace YAML
