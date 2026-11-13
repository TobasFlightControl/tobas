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

std::string textFromEnum(JointRole _role);
bool enumFromText(const std::string& _text, JointRole& _dst);

bool isServoJoint(JointRole _role);
}  // namespace tobas

namespace YAML
{
template <>
struct convert<tobas::JointRole>
{
  static Node encode(tobas::JointRole _rhs);
  static bool decode(const Node& _node, tobas::JointRole& _rhs);
};
}  // namespace YAML
