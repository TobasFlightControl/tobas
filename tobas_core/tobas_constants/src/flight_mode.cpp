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

std::string flightModeTextFromEnum(FlightMode mode)
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

std::expected<FlightMode, std::string> flightModeEnumFromText(const std::string& text)
{
  if (text == kAcrobatText) {
    return FlightMode::kAcrobat;
  }
  else if (text == kStabilizeText) {
    return FlightMode::kStabilize;
  }
  else if (text == kLoiterText) {
    return FlightMode::kLoiter;
  }
  else {
    return std::unexpected("Invalid flight mode string.");
  }
}
}  // namespace tobas

namespace YAML
{
Node convert<tobas::FlightMode>::encode(const tobas::FlightMode& rhs)
{
  Node node;
  node = tobas::flightModeTextFromEnum(rhs);
  return Node(tobas::flightModeTextFromEnum(rhs));
}

bool convert<tobas::FlightMode>::decode(const Node& node, tobas::FlightMode& rhs)
{
  if (!node.IsScalar()) {
    return false;
  }

  const auto result = tobas::flightModeEnumFromText(node.as<std::string>());
  if (!result) {
    return false;
  }

  rhs = *result;
  return true;
}
}  // namespace YAML
