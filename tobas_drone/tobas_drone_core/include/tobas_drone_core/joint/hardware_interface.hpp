#pragma once

#include <yaml-cpp/yaml.h>

namespace tobas
{
enum jnt_hw_iface_t : uint8_t
{
  PWM,
  OTHER,
};

/* jnt_hw_iface_t -> "position" or "velocity" or "effort" */
std::string jntHwIfaceEnumToText(jnt_hw_iface_t interface);

/* "position" or "velocity" or "effort" -> jnt_hw_iface_t */
jnt_hw_iface_t jntHwIfaceTextToEnum(const std::string& text);
}  // namespace tobas

namespace YAML
{
template <>
struct convert<tobas::jnt_hw_iface_t>
{
  static Node encode(const tobas::jnt_hw_iface_t& rhs)
  {
    return Node(static_cast<int>(rhs));
  }

  static bool decode(const Node& node, tobas::jnt_hw_iface_t& rhs)
  {
    if (!node.IsScalar())
      return false;

    rhs = static_cast<tobas::jnt_hw_iface_t>(node.as<int>());
    return true;
  }
};
}  // namespace YAML
