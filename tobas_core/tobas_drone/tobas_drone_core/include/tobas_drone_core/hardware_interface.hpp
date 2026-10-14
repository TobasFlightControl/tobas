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

std::string textFromEnum(HardwareInterface _value);
bool enumFromText(const std::string& _text, HardwareInterface& _dst);
}  // namespace tobas

namespace YAML
{
template <>
struct convert<tobas::HardwareInterface>
{
  static Node encode(tobas::HardwareInterface _rhs);
  static bool decode(const Node& _node, tobas::HardwareInterface& _rhs);
};
}  // namespace YAML
