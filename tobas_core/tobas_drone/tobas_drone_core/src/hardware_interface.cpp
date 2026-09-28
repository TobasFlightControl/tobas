// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_drone_core/hardware_interface.hpp"

#include <iostream>

namespace tobas
{
namespace
{
constexpr char kPwmText[] = "pwm";
constexpr char kOtherText[] = "other";
}  // namespace

std::string textFromEnum(HardwareInterface value)
{
  switch (value) {
    case HardwareInterface::kPwm:
      return kPwmText;
    case HardwareInterface::kOther:
      return kOtherText;
    default:
      throw;
  }
}

bool enumFromText(const std::string& text, HardwareInterface& dst)
{
  if (text == kPwmText) {
    dst = HardwareInterface::kPwm;
    return true;
  }
  else if (text == kOtherText) {
    dst = HardwareInterface::kOther;
    return true;
  }
  else {
    std::cerr << "Invalid joint hardware interface: " << text << std::endl;
    return false;
  }
}
}  // namespace tobas

namespace YAML
{
Node convert<tobas::HardwareInterface>::encode(const tobas::HardwareInterface& rhs)
{
  Node node;
  node = tobas::textFromEnum(rhs);
  return Node(tobas::textFromEnum(rhs));
}

bool convert<tobas::HardwareInterface>::decode(const Node& node, tobas::HardwareInterface& rhs)
{
  if (!node.IsScalar()) {
    return false;
  }

  return tobas::enumFromText(node.as<std::string>(), rhs);
}
}  // namespace YAML
