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

std::string textFromEnum(JointCommandInterface _value);
bool enumFromText(const std::string& _text, JointCommandInterface& _dst);
}  // namespace tobas

namespace YAML
{
template <>
struct convert<tobas::JointCommandInterface>
{
  static Node encode(tobas::JointCommandInterface _rhs);
  static bool decode(const Node& _node, tobas::JointCommandInterface& _rhs);
};
}  // namespace YAML
