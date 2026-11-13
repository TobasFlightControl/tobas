// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <yaml-cpp/yaml.h>

namespace tobas
{
enum class TurningDirection
{
  CCW,
  CW,
};

std::string textFromEnum(TurningDirection _interface);
bool enumFromText(const std::string& _text, TurningDirection& _dst);

/** CCW = 1, CW = -1 */
inline constexpr int sign(TurningDirection _direction)
{
  switch (_direction) {
    case TurningDirection::CCW:
      return 1;
    case TurningDirection::CW:
      return -1;
    default:
      throw;
  }
}
}  // namespace tobas

namespace YAML
{
template <>
struct convert<tobas::TurningDirection>
{
  static Node encode(tobas::TurningDirection _rhs);
  static bool decode(const Node& _node, tobas::TurningDirection& _rhs);
};
}  // namespace YAML
