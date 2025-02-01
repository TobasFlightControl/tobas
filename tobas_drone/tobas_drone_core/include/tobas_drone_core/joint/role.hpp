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

bool isServoJoint(jnt_role_t role);
}  // namespace tobas

namespace YAML
{
template <>
struct convert<tobas::jnt_role_t>
{
  static Node encode(const tobas::jnt_role_t& rhs)
  {
    return Node(static_cast<int>(rhs));
  }

  static bool decode(const Node& node, tobas::jnt_role_t& rhs)
  {
    if (!node.IsScalar())
      return false;

    rhs = static_cast<tobas::jnt_role_t>(node.as<int>());
    return true;
  }
};
}  // namespace YAML
