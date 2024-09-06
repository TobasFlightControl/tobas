#pragma once

#include <yaml-cpp/yaml.h>

namespace tobas
{
enum joint_control_type_t : uint8_t
{
  POSITION_CONTROL,
  VELOCITY_CONTROL,
  EFFORT_CONTROL,
};
}  // namespace tobas

namespace YAML
{
template <>
struct convert<tobas::joint_control_type_t>
{
  static Node encode(const tobas::joint_control_type_t& rhs)
  {
    return Node(static_cast<int>(rhs));
  }

  static bool decode(const Node& node, tobas::joint_control_type_t& rhs)
  {
    if (!node.IsScalar())
      return false;

    rhs = static_cast<tobas::joint_control_type_t>(node.as<int>());
    return true;
  }
};
}  // namespace YAML
