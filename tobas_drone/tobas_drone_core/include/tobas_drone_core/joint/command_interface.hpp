#pragma once

#include <yaml-cpp/yaml.h>

namespace tobas
{
enum struct jnt_cmd_iface_t : uint8_t
{
  POSITION,
  VELOCITY,
  EFFORT,
};

/* jnt_cmd_iface_t -> "position" or "velocity" or "effort" */
std::string jntCmdIfaceEnumToText(jnt_cmd_iface_t interface);

/* "position" or "velocity" or "effort" -> jnt_cmd_iface_t */
jnt_cmd_iface_t jntCmdIfaceTextToEnum(const std::string& text);
}  // namespace tobas

namespace YAML
{
template <>
struct convert<tobas::jnt_cmd_iface_t>
{
  static Node encode(const tobas::jnt_cmd_iface_t& rhs)
  {
    return Node(static_cast<int>(rhs));
  }

  static bool decode(const Node& node, tobas::jnt_cmd_iface_t& rhs)
  {
    if (!node.IsScalar())
      return false;

    rhs = static_cast<tobas::jnt_cmd_iface_t>(node.as<int>());
    return true;
  }
};
}  // namespace YAML
