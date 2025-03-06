#pragma once

#include <yaml-cpp/yaml.h>

namespace tobas
{
enum struct flight_mode_t : uint8_t
{
  ACROBAT,
  STABILIZE,
  LOITER,
};

std::string textFromEnum(flight_mode_t role);
bool enumFromText(const std::string& text, flight_mode_t& dst);
}  // namespace tobas

namespace YAML
{
template <>
struct convert<tobas::flight_mode_t>
{
  static Node encode(const tobas::flight_mode_t& rhs);
  static bool decode(const Node& node, tobas::flight_mode_t& rhs);
};
}  // namespace YAML
