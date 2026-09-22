// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_drone_core/fixed_wing/control_surface_type.hpp"

#include <iostream>

#define AILERON_TEXT "aileron"
#define ELEVATOR_TEXT "elevator"
#define RUDDER_TEXT "rudder"
#define OTHER_TEXT "other"

using namespace std;

namespace tobas
{
string textFromEnum(ControlSurfaceType cmd_iface)
{
  switch (cmd_iface) {
    case ControlSurfaceType::kAileron:
      return AILERON_TEXT;
    case ControlSurfaceType::kElevator:
      return ELEVATOR_TEXT;
    case ControlSurfaceType::kRudder:
      return RUDDER_TEXT;
    case ControlSurfaceType::kOther:
      return OTHER_TEXT;
    default:
      throw;
  }
}

bool enumFromText(const string& text, ControlSurfaceType& dst)
{
  if (text == AILERON_TEXT) {
    dst = ControlSurfaceType::kAileron;
    return true;
  }
  else if (text == ELEVATOR_TEXT) {
    dst = ControlSurfaceType::kElevator;
    return true;
  }
  else if (text == RUDDER_TEXT) {
    dst = ControlSurfaceType::kRudder;
    return true;
  }
  else if (text == OTHER_TEXT) {
    dst = ControlSurfaceType::kOther;
    return true;
  }
  else {
    cerr << "Invalid control surface type: " << text << endl;
    return false;
  }
}
}  // namespace tobas

namespace YAML
{
Node convert<tobas::ControlSurfaceType>::encode(const tobas::ControlSurfaceType& rhs)
{
  Node node;
  node = tobas::textFromEnum(rhs);
  return Node(tobas::textFromEnum(rhs));
}

bool convert<tobas::ControlSurfaceType>::decode(const Node& node, tobas::ControlSurfaceType& rhs)
{
  if (!node.IsScalar()) {
    return false;
  }

  return tobas::enumFromText(node.as<string>(), rhs);
}
}  // namespace YAML
