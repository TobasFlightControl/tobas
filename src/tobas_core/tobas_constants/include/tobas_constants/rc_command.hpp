// SPDX-License-Identifier: Apache-2.0
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <yaml-cpp/yaml.h>

namespace tobas
{
enum class RcCommand
{
  kRateThrottle,
  kRateThrottleVector,
  kAngleThrottle,
  kAngleThrottleVector,
  kAccelYaw,
  kAccelPitchYaw,
  kPosVelAccYaw,
  kPosVelAccPitchYaw,
  kAccelRate,
  kAccelAngle,
  kPosVelAccAngle,
  kSpeedRollDPitch,
};

std::string textFromEnum(RcCommand _cmd);
bool enumFromText(const std::string& _text, RcCommand& _dst);
}  // namespace tobas

namespace YAML
{
template <>
struct convert<tobas::RcCommand>
{
  static Node encode(tobas::RcCommand _rhs);
  static bool decode(const Node& _node, tobas::RcCommand& _rhs);
};
}  // namespace YAML
