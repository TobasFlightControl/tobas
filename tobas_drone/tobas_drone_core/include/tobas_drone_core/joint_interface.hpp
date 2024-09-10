#pragma once

#include <yaml-cpp/yaml.h>

namespace tobas
{
enum joint_interface_t : uint8_t
{
  POSITION,
  VELOCITY,
  EFFORT,
};

/* joint_interface_t -> "position" or "velocity" or "effort" */
std::string jointIFEnumToText(joint_interface_t interface);

/* "position" or "velocity" or "effort" -> joint_interface_t */
joint_interface_t jointIFTextToEnum(const std::string& text);
}  // namespace tobas

namespace YAML
{
template <>
struct convert<tobas::joint_interface_t>
{
  static Node encode(const tobas::joint_interface_t& rhs)
  {
    return Node(static_cast<int>(rhs));
  }

  static bool decode(const Node& node, tobas::joint_interface_t& rhs)
  {
    if (!node.IsScalar())
      return false;

    rhs = static_cast<tobas::joint_interface_t>(node.as<int>());
    return true;
  }
};
}  // namespace YAML
