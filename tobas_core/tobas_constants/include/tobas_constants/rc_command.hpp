#pragma once

#include <yaml-cpp/yaml.h>

namespace tobas
{
enum struct RcCommand
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

std::string textFromEnum(RcCommand cmd);
bool enumFromText(const std::string& text, RcCommand& dst);
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
