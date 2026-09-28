// SPDX-License-Identifier: Apache-2.0
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <expected>
#include <string>

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

std::string rcCommandTextFromEnum(RcCommand cmd);
std::expected<RcCommand, std::string> rcCommandEnumFromText(const std::string& text);
}  // namespace tobas

namespace YAML
{
template <>
struct convert<tobas::RcCommand>
{
  static Node encode(const tobas::RcCommand& rhs);
  static bool decode(const Node& node, tobas::RcCommand& rhs);
};
}  // namespace YAML
