#pragma once

#include <yaml-cpp/yaml.h>

namespace tobas
{
enum struct rc_command_t : uint8_t
{
  RATE_THROTTLE,
  ANGLE_THROTTLE,
  ACCEL_YAW,
  POS_VEL_YAW,
  ACCEL_RATE,
  ACCEL_ANGLE,
  POS_VEL_ANGLE,
  SPEED_ROLL_DPITCH,
};

std::string textFromEnum(rc_command_t role);
bool enumFromText(const std::string& text, rc_command_t& dst);
}  // namespace tobas

namespace YAML
{
template <>
struct convert<tobas::rc_command_t>
{
  static Node encode(const tobas::rc_command_t& rhs);
  static bool decode(const Node& node, tobas::rc_command_t& rhs);
};
}  // namespace YAML
