// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <yaml-cpp/yaml.h>

namespace tobas
{
namespace gui
{
namespace sa
{
enum struct FrameType
{
  kUndefined,
  kPlanarMulticopter,
  kNonPlanarMulticopter,
  kYAxisTiltMulticopter,
  kRandomAxisTiltMulticopter,
  kFixedWing,
};

std::string textFromEnum(FrameType arg);
bool enumFromText(const std::string& text, FrameType& dst);
}  // namespace sa
}  // namespace gui
}  // namespace tobas

namespace YAML
{
template <>
struct convert<tobas::gui::sa::FrameType>
{
  static Node encode(const tobas::gui::sa::FrameType& rhs);
  static bool decode(const Node& node, tobas::gui::sa::FrameType& rhs);
};
}  // namespace YAML
