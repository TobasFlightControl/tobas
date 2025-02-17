#pragma once

#include <yaml-cpp/yaml.h>

namespace tobas
{
enum struct jnt_cmd_iface_t : uint8_t
{
  POSITION,
  VELOCITY,
  EFFORT,
  NONE,
};

std::string textFromEnum(jnt_cmd_iface_t interface);
bool enumFromText(const std::string& text, jnt_cmd_iface_t& dst);
}  // namespace tobas

namespace YAML
{
template <>
struct convert<tobas::jnt_cmd_iface_t>
{
  static Node encode(const tobas::jnt_cmd_iface_t& rhs);
  static bool decode(const Node& node, tobas::jnt_cmd_iface_t& rhs);
};
}  // namespace YAML
