// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_drone_core/joint/role.hpp"

namespace tobas
{
namespace
{
constexpr char kTiltJoint[] = "tilt_joint";
constexpr char kControlSurface[] = "control_surface";
constexpr char kUserActive[] = "user_active";
constexpr char kUserPassive[] = "user_passive";
}  // namespace

std::string textFromEnum(JointRole _role)
{
  switch (_role) {
    case JointRole::kTiltJoint:
      return kTiltJoint;
    case JointRole::kControlSurface:
      return kControlSurface;
    case JointRole::kUserActive:
      return kUserActive;
    case JointRole::kUserPassive:
      return kUserPassive;
    default:
      throw;
  }
}

bool enumFromText(const std::string& _text, JointRole& _dst)
{
  if (_text == kTiltJoint) {
    _dst = JointRole::kTiltJoint;
    return true;
  }
  else if (_text == kControlSurface) {
    _dst = JointRole::kControlSurface;
    return true;
  }
  else if (_text == kUserActive) {
    _dst = JointRole::kUserActive;
    return true;
  }
  else if (_text == kUserPassive) {
    _dst = JointRole::kUserPassive;
    return true;
  }
  else {
    return false;
  }
}

bool isServoJoint(JointRole _role)
{
  switch (_role) {
    case JointRole::kTiltJoint:
      return true;
    case JointRole::kControlSurface:
      return true;
    case JointRole::kUserActive:
      return true;
    case JointRole::kUserPassive:
      return false;
    default:
      throw;
  }
}
}  // namespace tobas

namespace YAML
{
Node convert<tobas::JointRole>::encode(tobas::JointRole _rhs)
{
  Node node;
  node = tobas::textFromEnum(_rhs);
  return Node(tobas::textFromEnum(_rhs));
}

bool convert<tobas::JointRole>::decode(const Node& _node, tobas::JointRole& _rhs)
{
  if (!_node.IsScalar()) {
    return false;
  }

  return tobas::enumFromText(_node.as<std::string>(), _rhs);
}
}  // namespace YAML
