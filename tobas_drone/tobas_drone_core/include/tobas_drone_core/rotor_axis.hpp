#pragma once

#include <cinttypes>
#include <yaml-cpp/yaml.h>

namespace tobas
{
enum rotor_axis_t : uint8_t
{
  X_POSITIVE,
  Z_POSITIVE,
  UNKNOWN,
};
}  // namespace tobas

namespace YAML
{
template <>
struct convert<tobas::rotor_axis_t>
{
  static Node encode(const tobas::rotor_axis_t& rhs)
  {
    Node node;

    switch (rhs)
    {
      case tobas::rotor_axis_t::X_POSITIVE:
        node = "X_POSITIVE";
        break;
      case tobas::rotor_axis_t::Z_POSITIVE:
        node = "Z_POSITIVE";
        break;
      case tobas::rotor_axis_t::UNKNOWN:
        node = "UNKNOWN";
        break;
    }

    return node;
  }

  static bool decode(const Node& node, tobas::rotor_axis_t& rhs)
  {
    if (!node.IsScalar())
      return false;

    const auto value = node.as<std::string>();
    if (value == "X_POSITIVE")
      rhs = tobas::rotor_axis_t::X_POSITIVE;
    else if (value == "Z_POSITIVE")
      rhs = tobas::rotor_axis_t::Z_POSITIVE;
    else if (value == "UNKNOWN")
      rhs = tobas::rotor_axis_t::UNKNOWN;
    else
      return false;

    return true;
  }
};
}  // namespace YAML
