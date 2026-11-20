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
string textFromEnum(ControlSurfaceType _cmd_iface)
{
  switch (_cmd_iface) {
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

bool enumFromText(const string& _text, ControlSurfaceType& _dst)
{
  if (_text == AILERON_TEXT) {
    _dst = ControlSurfaceType::kAileron;
    return true;
  }
  else if (_text == ELEVATOR_TEXT) {
    _dst = ControlSurfaceType::kElevator;
    return true;
  }
  else if (_text == RUDDER_TEXT) {
    _dst = ControlSurfaceType::kRudder;
    return true;
  }
  else if (_text == OTHER_TEXT) {
    _dst = ControlSurfaceType::kOther;
    return true;
  }
  else {
    cerr << "Invalid control surface type: " << _text << endl;
    return false;
  }
}
}  // namespace tobas

namespace YAML
{
Node convert<tobas::ControlSurfaceType>::encode(const tobas::ControlSurfaceType& _rhs)
{
  Node node;
  node = tobas::textFromEnum(_rhs);
  return Node(tobas::textFromEnum(_rhs));
}

bool convert<tobas::ControlSurfaceType>::decode(const Node& _node, tobas::ControlSurfaceType& _rhs)
{
  if (!_node.IsScalar()) {
    return false;
  }

  return tobas::enumFromText(_node.as<string>(), _rhs);
}
}  // namespace YAML
