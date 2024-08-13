#pragma once

#include <yaml-cpp/yaml.h>

namespace tobas
{
enum joint_control_type_t : uint8_t
{
  POSITION_CONTROL,
  VELOCITY_CONTROL,
  EFFORT_CONTROL,
};

namespace joint_control_type
{
static constexpr char kPosCtrlName[] = "POSITION_CONTROL";
static constexpr char kVelCtrlName[] = "VELOCITY_CONTROL";
static constexpr char kEffCtrlName[] = "EFFORT_CONTROL";
}  // namespace joint_control_type
}  // namespace tobas

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
        node = tobas::joint_control_type::kPosCtrlName;
        break;
      case tobas::joint_control_type_t::VELOCITY_CONTROL:
        node = tobas::joint_control_type::kVelCtrlName;
        break;
      case tobas::joint_control_type_t::EFFORT_CONTROL:
        node = tobas::joint_control_type::kEffCtrlName;
        break;
    }

    return node;
  }

  static bool decode(const Node& node, tobas::joint_control_type_t& rhs)
  {
    if (!node.IsScalar())
      return false;

    const auto value = node.as<std::string>();
    if (value == tobas::joint_control_type::kPosCtrlName)
      rhs = tobas::joint_control_type_t::POSITION_CONTROL;
    else if (value == tobas::joint_control_type::kVelCtrlName)
      rhs = tobas::joint_control_type_t::VELOCITY_CONTROL;
    else if (value == tobas::joint_control_type::kEffCtrlName)
      rhs = tobas::joint_control_type_t::EFFORT_CONTROL;
    else
      return false;

    return true;
  }
};
}  // namespace YAML
