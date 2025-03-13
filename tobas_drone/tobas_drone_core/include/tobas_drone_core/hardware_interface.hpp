#pragma once

#include <yaml-cpp/yaml.h>

namespace tobas
{
enum struct hw_iface_t : uint8_t
{
  PWM,
  OTHER,
};

std::string textFromEnum(hw_iface_t value);
bool enumFromText(const std::string& text, hw_iface_t& dst);
}  // namespace tobas

namespace YAML
{
template <>
struct convert<tobas::hw_iface_t>
{
  static Node encode(const tobas::hw_iface_t& rhs);
  static bool decode(const Node& node, tobas::hw_iface_t& rhs);
};
}  // namespace YAML
