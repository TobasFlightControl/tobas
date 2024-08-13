#pragma once

#include <yaml-cpp/yaml.h>

namespace tobas
{
enum turning_direction_t : uint8_t
{
  CCW,
  CW,
};

namespace turning_direction
{
static constexpr char kCCWName[] = "CCW";
static constexpr char kCWName[] = "CW";
}  // namespace turning_direction
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
        node = tobas::turning_direction::kCCWName;
        break;
      case tobas::turning_direction_t::CW:
        node = tobas::turning_direction::kCWName;
        break;
    }

    return node;
  }

  static bool decode(const Node& node, tobas::turning_direction_t& rhs)
  {
    if (!node.IsScalar())
      return false;

    const auto value = node.as<std::string>();
    if (value == tobas::turning_direction::kCCWName)
      rhs = tobas::turning_direction_t::CCW;
    else if (value == tobas::turning_direction::kCWName)
      rhs = tobas::turning_direction_t::CW;
    else
      return false;

    return true;
  }
};
}  // namespace YAML
