#pragma once

#include <yaml-cpp/yaml.h>

namespace tobas
{
enum struct jnt_hw_iface_t : uint8_t
{
  PWM,
  OTHER,
};

std::string textFromEnum(jnt_hw_iface_t interface);
bool enumFromText(const std::string& text, jnt_hw_iface_t& dst);
}  // namespace tobas

namespace YAML
{
template <>
struct convert<tobas::jnt_hw_iface_t>
{
  static Node encode(const tobas::jnt_hw_iface_t& rhs);
  static bool decode(const Node& node, tobas::jnt_hw_iface_t& rhs);
};
}  // namespace YAML
