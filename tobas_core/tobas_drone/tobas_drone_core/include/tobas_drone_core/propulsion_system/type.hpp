#pragma once

#include <yaml-cpp/yaml.h>

namespace tobas
{
enum struct PropulsionSystem
{
  kElectric,
  kIce,
};

std::string textFromEnum(PropulsionSystem interface);
bool enumFromText(const std::string& text, PropulsionSystem& dst);
}  // namespace tobas

namespace YAML
{
template <>
struct convert<tobas::PropulsionSystem>
{
  static Node encode(const tobas::PropulsionSystem& rhs);
  static bool decode(const Node& node, tobas::PropulsionSystem& rhs);
};
}  // namespace YAML
