#pragma once

#include <yaml-cpp/yaml.h>

namespace tobas
{
enum turning_direction_t : uint8_t
{
  CCW,
  CW,
};

/* CCW = 1, CW = -1 */
constexpr inline int sign(turning_direction_t direction)
{
  switch (direction)
  {
    case turning_direction_t::CCW:
      return 1;
    case turning_direction_t::CW:
      return -1;
    default:
      throw;
  }
}
}  // namespace tobas

namespace YAML
{
template <>
struct convert<tobas::turning_direction_t>
{
  static Node encode(const tobas::turning_direction_t& rhs)
  {
    return Node(static_cast<int>(rhs));
  }

  static bool decode(const Node& node, tobas::turning_direction_t& rhs)
  {
    if (!node.IsScalar())
      return false;

    rhs = static_cast<tobas::turning_direction_t>(node.as<int>());
    return true;
  }
};
}  // namespace YAML
