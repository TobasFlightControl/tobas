// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <yaml-cpp/yaml.h>

namespace tobas
{
enum class PropulsionSystem
{
  kElectric,
  kIce,
};

std::string textFromEnum(PropulsionSystem _interface);
bool enumFromText(const std::string& _text, PropulsionSystem& _dst);
}  // namespace tobas

namespace YAML
{
template <>
struct convert<tobas::PropulsionSystem>
{
  static Node encode(tobas::PropulsionSystem _rhs);
  static bool decode(const Node& _node, tobas::PropulsionSystem& _rhs);
};
}  // namespace YAML
