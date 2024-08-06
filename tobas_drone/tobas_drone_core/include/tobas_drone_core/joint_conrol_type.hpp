#pragma once

#include <yaml-cpp/yaml.h>

namespace tobas
{
enum struct joint_control_type_t : uint8_t
{
  POSITION_CONTROL,
  VELOCITY_CONTROL,
  EFFORT_CONTROL,
};
}

namespace YAML
{
template <>
struct convert<tobas::joint_control_type_t>
{
  static Node encode(const tobas::joint_control_type_t& rhs)
  {
    Node node;

    switch (rhs)
    {
      case tobas::joint_control_type_t::POSITION_CONTROL:
        node = "POSITION_CONTROL";
        break;
      case tobas::joint_control_type_t::VELOCITY_CONTROL:
        node = "VELOCITY_CONTROL";
        break;
      case tobas::joint_control_type_t::EFFORT_CONTROL:
        node = "EFFORT_CONTROL";
        break;
    }

    return node;
  }

  static bool decode(const Node& node, tobas::joint_control_type_t& rhs)
  {
    if (!node.IsScalar())
      return false;

    const auto value = node.as<std::string>();
    if (value == "POSITION_CONTROL")
      rhs = tobas::joint_control_type_t::POSITION_CONTROL;
    else if (value == "VELOCITY_CONTROL")
      rhs = tobas::joint_control_type_t::VELOCITY_CONTROL;
    else if (value == "EFFORT_CONTROL")
      rhs = tobas::joint_control_type_t::EFFORT_CONTROL;
    else
      return false;

    return true;
  }
};
}  // namespace YAML
