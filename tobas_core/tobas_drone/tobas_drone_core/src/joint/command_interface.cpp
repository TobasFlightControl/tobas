// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_drone_core/joint/command_interface.hpp"

namespace tobas
{
namespace
{
constexpr char kPosition[] = "position";
constexpr char kVelocity[] = "velocity";
constexpr char kEffort[] = "effort";
constexpr char kNone[] = "none";
}  // namespace

std::string textFromEnum(JointCommandInterface _value)
{
  switch (_value) {
    case JointCommandInterface::kPosition:
      return kPosition;
    case JointCommandInterface::kVelocity:
      return kVelocity;
    case JointCommandInterface::kEffort:
      return kEffort;
    case JointCommandInterface::kNone:
      return kNone;
    default:
      throw;
  }
}

bool enumFromText(const std::string& _text, JointCommandInterface& _dst)
{
  if (_text == kPosition) {
    _dst = JointCommandInterface::kPosition;
    return true;
  }
  else if (_text == kVelocity) {
    _dst = JointCommandInterface::kVelocity;
    return true;
  }
  else if (_text == kEffort) {
    _dst = JointCommandInterface::kEffort;
    return true;
  }
  else if (_text == kNone) {
    _dst = JointCommandInterface::kNone;
    return true;
  }
  else {
    return false;
  }
}
}  // namespace tobas

namespace YAML
{
Node convert<tobas::JointCommandInterface>::encode(tobas::JointCommandInterface _rhs)
{
  Node node;
  node = tobas::textFromEnum(_rhs);
  return Node(tobas::textFromEnum(_rhs));
}

bool convert<tobas::JointCommandInterface>::decode(const Node& _node, tobas::JointCommandInterface& _rhs)
{
  if (!_node.IsScalar()) {
    return false;
  }

  return tobas::enumFromText(_node.as<std::string>(), _rhs);
}
}  // namespace YAML
