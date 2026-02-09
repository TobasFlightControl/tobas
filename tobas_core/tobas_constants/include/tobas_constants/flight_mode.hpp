#pragma once

#include <yaml-cpp/yaml.h>

namespace tobas
{
enum struct FlightMode
{
  kAcrobat,
  kStabilize,
  kLoiter,
};

std::string textFromEnum(FlightMode mode);
bool enumFromText(const std::string& text, FlightMode& dst);
}  // namespace tobas

namespace YAML
{
template <>
struct convert<tobas::FlightMode>
{
  static Node encode(const tobas::FlightMode& rhs);
  static bool decode(const Node& node, tobas::FlightMode& rhs);
};
}  // namespace YAML
