// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_drone_core/hardware_interface.hpp"

namespace tobas
{
namespace
{
constexpr char kPwmText[] = "pwm";
constexpr char kOtherText[] = "other";
}  // namespace

std::string textFromEnum(HardwareInterface _value)
{
  switch (_value) {
    case HardwareInterface::kPwm:
      return kPwmText;
    case HardwareInterface::kOther:
      return kOtherText;
    default:
      throw;
  }
}

bool enumFromText(const std::string& _text, HardwareInterface& _dst)
{
  if (_text == kPwmText) {
    _dst = HardwareInterface::kPwm;
    return true;
  }
  else if (_text == kOtherText) {
    _dst = HardwareInterface::kOther;
    return true;
  }
  else {
    return false;
  }
}
}  // namespace tobas

namespace YAML
{
Node convert<tobas::HardwareInterface>::encode(tobas::HardwareInterface _rhs)
{
  Node node;
  node = tobas::textFromEnum(_rhs);
  return Node(tobas::textFromEnum(_rhs));
}

bool convert<tobas::HardwareInterface>::decode(const Node& _node, tobas::HardwareInterface& _rhs)
{
  if (!_node.IsScalar()) {
    return false;
  }

  return tobas::enumFromText(_node.as<std::string>(), _rhs);
}
}  // namespace YAML
