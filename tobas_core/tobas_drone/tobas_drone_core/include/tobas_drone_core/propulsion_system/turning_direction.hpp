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

std::string textFromEnum(TurningDirection interface);
bool enumFromText(const std::string& text, TurningDirection& dst);

/* CCW = 1, CW = -1 */
inline constexpr int sign(TurningDirection direction)
{
  switch (direction) {
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
  static Node encode(const tobas::TurningDirection& rhs);
  static bool decode(const Node& node, tobas::TurningDirection& rhs);
};
}  // namespace YAML
