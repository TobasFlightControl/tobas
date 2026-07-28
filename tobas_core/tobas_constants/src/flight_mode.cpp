// SPDX-License-Identifier: Apache-2.0
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_constants/flight_mode.hpp"

#include <iostream>

#define ACROBAT_TEXT "acrobat"
#define STABILIZE_TEXT "stabilize"
#define LOITER_TEXT "loiter"

using namespace std;

namespace tobas
{
string textFromEnum(FlightMode mode)
{
  switch (mode) {
    case FlightMode::kAcrobat:
      return ACROBAT_TEXT;
    case FlightMode::kStabilize:
      return STABILIZE_TEXT;
    case FlightMode::kLoiter:
      return LOITER_TEXT;
    default:
      throw;
  }
}

bool enumFromText(const string& text, FlightMode& dst)
{
  if (text == ACROBAT_TEXT) {
    dst = FlightMode::kAcrobat;
    return true;
  }
  else if (text == STABILIZE_TEXT) {
    dst = FlightMode::kStabilize;
    return true;
  }
  else if (text == LOITER_TEXT) {
    dst = FlightMode::kLoiter;
    return true;
  }
  else {
    cerr << "Invalid flight mode: " << text << endl;
    return false;
  }
}
}  // namespace tobas

namespace YAML
{
Node convert<tobas::FlightMode>::encode(const tobas::FlightMode& rhs)
{
  Node node;
  node = tobas::textFromEnum(rhs);
  return Node(tobas::textFromEnum(rhs));
}

bool convert<tobas::FlightMode>::decode(const Node& node, tobas::FlightMode& rhs)
{
  if (!node.IsScalar()) {
    return false;
  }

  return tobas::enumFromText(node.as<string>(), rhs);
}
}  // namespace YAML
