// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_drone_core/propulsion_system/turning_direction.hpp"

#include <iostream>

#define CCW_TEXT "ccw"
#define CW_TEXT "cw"

using namespace std;

namespace tobas
{
string textFromEnum(TurningDirection cmd_iface)
{
  switch (cmd_iface) {
    case TurningDirection::CCW:
      return CCW_TEXT;
    case TurningDirection::CW:
      return CW_TEXT;
    default:
      throw;
  }
}

bool enumFromText(const string& text, TurningDirection& dst)
{
  if (text == CCW_TEXT) {
    dst = TurningDirection::CCW;
    return true;
  }
  else if (text == CW_TEXT) {
    dst = TurningDirection::CW;
    return true;
  }
  else {
    cerr << "Invalid rotor turning direction: " << text << endl;
    return false;
  }
}
}  // namespace tobas

namespace YAML
{
Node convert<tobas::TurningDirection>::encode(const tobas::TurningDirection& rhs)
{
  Node node;
  node = tobas::textFromEnum(rhs);
  return Node(tobas::textFromEnum(rhs));
}

bool convert<tobas::TurningDirection>::decode(const Node& node, tobas::TurningDirection& rhs)
{
  if (!node.IsScalar()) {
    return false;
  }

  return tobas::enumFromText(node.as<string>(), rhs);
}
}  // namespace YAML
