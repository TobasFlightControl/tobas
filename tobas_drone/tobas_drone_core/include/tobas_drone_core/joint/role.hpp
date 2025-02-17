#pragma once

#include <yaml-cpp/yaml.h>

namespace tobas
{
enum struct jnt_role_t : uint8_t
{
  ROTOR,
  TILT_JOINT,
  CONTROL_SURFACE,
  MANIPULATION,
  PASSIVE_WHEEL,
  OTHER,
};

std::string textFromEnum(jnt_role_t role);
bool enumFromText(const std::string& text, jnt_role_t& dst);

bool isServoJoint(jnt_role_t role);
}  // namespace tobas

namespace YAML
{
template <>
struct convert<tobas::jnt_role_t>
{
  static Node encode(const tobas::jnt_role_t& rhs);
  static bool decode(const Node& node, tobas::jnt_role_t& rhs);
};
}  // namespace YAML
