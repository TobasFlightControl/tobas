#pragma once

#include <yaml-cpp/yaml.h>

namespace tobas
{
enum struct propulsion_system_t : uint8_t
{
  ELECTRIC,
  ICE,
};

std::string textFromEnum(propulsion_system_t interface);
bool enumFromText(const std::string& text, propulsion_system_t& dst);
}  // namespace tobas

namespace YAML
{
template <>
struct convert<tobas::propulsion_system_t>
{
  static Node encode(const tobas::propulsion_system_t& rhs);
  static bool decode(const Node& node, tobas::propulsion_system_t& rhs);
};
}  // namespace YAML
