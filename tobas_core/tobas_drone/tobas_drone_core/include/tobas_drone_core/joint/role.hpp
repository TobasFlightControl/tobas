// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <yaml-cpp/yaml.h>

namespace tobas
{
enum class JointRole
{
  kTiltJoint,
  kControlSurface,
  kUserActive,
  kUserPassive,
};

std::string textFromEnum(JointRole role);
bool enumFromText(const std::string& text, JointRole& dst);

bool isServoJoint(JointRole role);
}  // namespace tobas

namespace YAML
{
template <>
struct convert<tobas::JointRole>
{
  static Node encode(const tobas::JointRole& rhs);
  static bool decode(const Node& node, tobas::JointRole& rhs);
};
}  // namespace YAML
