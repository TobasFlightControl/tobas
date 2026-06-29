// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <yaml-cpp/yaml.h>

namespace tobas
{
enum class HardwareInterface
{
  kPwm,
  kOther,
};

std::string textFromEnum(HardwareInterface value);
bool enumFromText(const std::string& text, HardwareInterface& dst);
}  // namespace tobas

namespace YAML
{
template <>
struct convert<tobas::HardwareInterface>
{
  static Node encode(const tobas::HardwareInterface& rhs);
  static bool decode(const Node& node, tobas::HardwareInterface& rhs);
};
}  // namespace YAML
