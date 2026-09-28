// SPDX-License-Identifier: Apache-2.0
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <expected>
#include <string>

#include <yaml-cpp/yaml.h>

namespace tobas
{
enum class FlightMode
{
  kAcrobat,
  kStabilize,
  kLoiter,
};

std::string flightModeTextFromEnum(FlightMode mode);
std::expected<FlightMode, std::string> flightModeEnumFromText(const std::string& text);
}  // namespace tobas

namespace YAML
{
template <>
struct convert<tobas::FlightMode>
{
  static Node encode(const tobas::FlightMode& rhs);
  static bool decode(const Node& node, tobas::FlightMode& rhs);
};
}  // namespace YAML
