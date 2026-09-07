// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_drone_core/joint/command_interface.hpp"

#include <iostream>

using namespace std;

namespace tobas
{
namespace
{
constexpr char kPosition[] = "position";
constexpr char kVelocity[] = "velocity";
constexpr char kEffort[] = "effort";
constexpr char kNone[] = "none";
}  // namespace

string textFromEnum(JointCommandInterface value)
{
  switch (value) {
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

bool enumFromText(const string& text, JointCommandInterface& dst)
{
  if (text == kPosition) {
    dst = JointCommandInterface::kPosition;
    return true;
  }
  else if (text == kVelocity) {
    dst = JointCommandInterface::kVelocity;
    return true;
  }
  else if (text == kEffort) {
    dst = JointCommandInterface::kEffort;
    return true;
  }
  else if (text == kNone) {
    dst = JointCommandInterface::kNone;
    return true;
  }
  else {
    cerr << "Invalid joint command interface: " << text << endl;
    return false;
  }
}
}  // namespace tobas

namespace YAML
{
Node convert<tobas::JointCommandInterface>::encode(const tobas::JointCommandInterface& rhs)
{
  Node node;
  node = tobas::textFromEnum(rhs);
  return Node(tobas::textFromEnum(rhs));
}

bool convert<tobas::JointCommandInterface>::decode(const Node& node, tobas::JointCommandInterface& rhs)
{
  if (!node.IsScalar()) {
    return false;
  }

  return tobas::enumFromText(node.as<string>(), rhs);
}
}  // namespace YAML
