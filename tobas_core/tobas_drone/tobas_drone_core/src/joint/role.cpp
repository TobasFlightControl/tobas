// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_drone_core/joint/role.hpp"

#include <iostream>

using namespace std;

namespace tobas
{
namespace
{
constexpr char kTiltJoint[] = "tilt_joint";
constexpr char kControlSurface[] = "control_surface";
constexpr char kUserActive[] = "user_active";
constexpr char kUserPassive[] = "user_passive";
}  // namespace

string textFromEnum(JointRole role)
{
  switch (role) {
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

bool enumFromText(const string& text, JointRole& dst)
{
  if (text == kTiltJoint) {
    dst = JointRole::kTiltJoint;
    return true;
  }
  else if (text == kControlSurface) {
    dst = JointRole::kControlSurface;
    return true;
  }
  else if (text == kUserActive) {
    dst = JointRole::kUserActive;
    return true;
  }
  else if (text == kUserPassive) {
    dst = JointRole::kUserPassive;
    return true;
  }
  else {
    cerr << "Invalid joint role: " << text << endl;
    return false;
  }
}

bool isServoJoint(JointRole role)
{
  switch (role) {
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
Node convert<tobas::JointRole>::encode(const tobas::JointRole& rhs)
{
  Node node;
  node = tobas::textFromEnum(rhs);
  return Node(tobas::textFromEnum(rhs));
}

bool convert<tobas::JointRole>::decode(const Node& node, tobas::JointRole& rhs)
{
  if (!node.IsScalar()) {
    return false;
  }

  return tobas::enumFromText(node.as<string>(), rhs);
}
}  // namespace YAML
