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

namespace rotor_axis
{
static constexpr char kXPositiveName[] = "X_POSITIVE";
static constexpr char kZPositiveName[] = "Z_POSITIVE";
static constexpr char kUnknownName[] = "UNKNOWN";
}  // namespace rotor_axis
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
        node = tobas::rotor_axis::kXPositiveName;
        break;
      case tobas::rotor_axis_t::Z_POSITIVE:
        node = tobas::rotor_axis::kZPositiveName;
        break;
      case tobas::rotor_axis_t::UNKNOWN:
        node = tobas::rotor_axis::kUnknownName;
        break;
    }

    return node;
  }

  static bool decode(const Node& node, tobas::rotor_axis_t& rhs)
  {
    if (!node.IsScalar())
      return false;

    const auto value = node.as<std::string>();
    if (value == tobas::rotor_axis::kXPositiveName)
      rhs = tobas::rotor_axis_t::X_POSITIVE;
    else if (value == tobas::rotor_axis::kZPositiveName)
      rhs = tobas::rotor_axis_t::Z_POSITIVE;
    else if (value == tobas::rotor_axis::kUnknownName)
      rhs = tobas::rotor_axis_t::UNKNOWN;
    else
      return false;

    return true;
  }
};
}  // namespace YAML
