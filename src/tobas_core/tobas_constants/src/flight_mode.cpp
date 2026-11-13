// SPDX-License-Identifier: Apache-2.0
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_constants/flight_mode.hpp"

namespace tobas
{
namespace
{
constexpr char kAcrobatText[] = "acrobat";
constexpr char kStabilizeText[] = "stabilize";
constexpr char kLoiterText[] = "loiter";
}  // namespace

std::string textFromEnum(FlightMode _mode)
{
  switch (_mode) {
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

bool enumFromText(const std::string& _text, FlightMode& _dst)
{
  if (_text == kAcrobatText) {
    _dst = FlightMode::kAcrobat;
    return true;
  }
  else if (_text == kStabilizeText) {
    _dst = FlightMode::kStabilize;
    return true;
  }
  else if (_text == kLoiterText) {
    _dst = FlightMode::kLoiter;
    return true;
  }
  else {
    return false;
  }
}
}  // namespace tobas

namespace YAML
{
Node convert<tobas::FlightMode>::encode(tobas::FlightMode _rhs)
{
  Node node;
  node = tobas::textFromEnum(_rhs);
  return Node(tobas::textFromEnum(_rhs));
}

bool convert<tobas::FlightMode>::decode(const Node& _node, tobas::FlightMode& _rhs)
{
  if (!_node.IsScalar()) {
    return false;
  }

  return tobas::enumFromText(_node.as<std::string>(), _rhs);
}
}  // namespace YAML
