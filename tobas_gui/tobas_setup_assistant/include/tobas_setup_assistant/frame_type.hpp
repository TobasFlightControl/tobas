// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <yaml-cpp/yaml.h>

namespace tobas::gui::sa
{
enum class FrameType
{
  kUndefined,
  kPlanarMulticopter,
  kNonPlanarMulticopter,
  kYAxisTiltMulticopter,
  kRandomAxisTiltMulticopter,
  kFixedWing,
};

std::string textFromEnum(FrameType _arg);
bool enumFromText(const std::string& _text, FrameType& _dst);
}  // namespace tobas::gui::sa

namespace YAML
{
template <>
struct convert<tobas::gui::sa::FrameType>
{
  static Node encode(tobas::gui::sa::FrameType _rhs);
  static bool decode(const Node& _node, tobas::gui::sa::FrameType& _rhs);
};
}  // namespace YAML
