#pragma once

#include <yaml-cpp/yaml.h>

namespace tobas
{
enum struct RcCommand
{
  kRateThrottle,
  kAngleThrottle,
  kAccelYaw,
  kAccelPitchYaw,
  kPosVelYaw,
  kPosVelPitchYaw,
  kAccelRate,
  kAccelAngle,
  kPosVelAngle,
  kSpeedRollDPitch,
};

std::string textFromEnum(RcCommand role);
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
