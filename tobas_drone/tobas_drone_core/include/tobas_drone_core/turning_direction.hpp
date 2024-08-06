#pragma once

#include <yaml-cpp/yaml.h>

namespace tobas
{
enum turning_direction_t : uint8_t
{
  CCW,
  CW,
};
}  // namespace tobas

namespace YAML
{
template <>
struct convert<tobas::turning_direction_t>
{
  static Node encode(const tobas::turning_direction_t& rhs)
  {
    Node node;

    switch (rhs)
    {
      case tobas::turning_direction_t::CCW:
        node = "CCW";
        break;
      case tobas::turning_direction_t::CW:
        node = "CW";
        break;
    }

    return node;
  }

  static bool decode(const Node& node, tobas::turning_direction_t& rhs)
  {
    if (!node.IsScalar())
      return false;

    const auto value = node.as<std::string>();
    if (value == "CCW")
      rhs = tobas::turning_direction_t::CCW;
    else if (value == "CW")
      rhs = tobas::turning_direction_t::CW;
    else
      return false;

    return true;
  }
};
}  // namespace YAML
