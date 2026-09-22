// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <yaml-cpp/yaml.h>

namespace tobas
{
enum class ControlSurfaceType
{
  kAileron,
  kElevator,
  kRudder,
  kOther,
};

std::string textFromEnum(ControlSurfaceType interface);
bool enumFromText(const std::string& text, ControlSurfaceType& dst);
}  // namespace tobas

namespace YAML
{
template <>
struct convert<tobas::ControlSurfaceType>
{
  static Node encode(const tobas::ControlSurfaceType& rhs);
  static bool decode(const Node& node, tobas::ControlSurfaceType& rhs);
};
}  // namespace YAML
