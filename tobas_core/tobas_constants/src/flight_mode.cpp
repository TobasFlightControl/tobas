// SPDX-License-Identifier: Apache-2.0
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_constants/flight_mode.hpp"

#include <iostream>

using namespace std;

namespace tobas
{
namespace
{
constexpr char kAcrobatText[] = "acrobat";
constexpr char kStabilizeText[] = "stabilize";
constexpr char kLoiterText[] = "loiter";
}  // namespace

string textFromEnum(FlightMode mode)
{
  switch (mode) {
    case FlightMode::kAcrobat:
      return kAcrobatText;
    case FlightMode::kStabilize:
      return kStabilizeText;
    case FlightMode::kLoiter:
      return kLoiterText;
    default:
      throw;
  }
}

bool enumFromText(const string& text, FlightMode& dst)
{
  if (text == kAcrobatText) {
    dst = FlightMode::kAcrobat;
    return true;
  }
  else if (text == kStabilizeText) {
    dst = FlightMode::kStabilize;
    return true;
  }
  else if (text == kLoiterText) {
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
