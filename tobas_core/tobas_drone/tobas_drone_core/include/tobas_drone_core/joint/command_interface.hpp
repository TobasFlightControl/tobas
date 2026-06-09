// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <yaml-cpp/yaml.h>

namespace tobas
{
enum class JointCommandInterface
{
  kPosition,
  kVelocity,
  kEffort,
  kNone,
};

std::string textFromEnum(JointCommandInterface value);
bool enumFromText(const std::string& text, JointCommandInterface& dst);
}  // namespace tobas

namespace YAML
{
template <>
struct convert<tobas::JointCommandInterface>
{
  static Node encode(const tobas::JointCommandInterface& rhs);
  static bool decode(const Node& node, tobas::JointCommandInterface& rhs);
};
}  // namespace YAML
