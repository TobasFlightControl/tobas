// SPDX-License-Identifier: Apache-2.0
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <yaml-cpp/yaml.h>

namespace tobas
{
enum class FlightMode
{
  kAcrobat,
  kStabilize,
  kLoiter,
};

std::string textFromEnum(FlightMode _mode);
bool enumFromText(const std::string& _text, FlightMode& _dst);
}  // namespace tobas

namespace YAML
{
template <>
struct convert<tobas::FlightMode>
{
  static Node encode(tobas::FlightMode _rhs);
  static bool decode(const Node& _node, tobas::FlightMode& _rhs);
};
}  // namespace YAML
