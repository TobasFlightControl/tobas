#pragma once

#include <yaml-cpp/yaml.h>

namespace tobas
{
enum joint_role_t : uint8_t
{
  MANIPULATION,
  TILT_ROTOR,
  CONTROL_SURFACE,
};
}  // namespace tobas

namespace YAML
{
template <>
struct convert<tobas::joint_role_t>
{
  static Node encode(const tobas::joint_role_t& rhs)
  {
    return Node(static_cast<int>(rhs));
  }

  static bool decode(const Node& node, tobas::joint_role_t& rhs)
  {
    if (!node.IsScalar())
      return false;

    rhs = static_cast<tobas::joint_role_t>(node.as<int>());
    return true;
  }
};
}  // namespace YAML
