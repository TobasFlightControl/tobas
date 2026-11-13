// SPDX-License-Identifier: Apache-2.0
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <yaml-cpp/yaml.h>

namespace tobas::mission
{
enum AltitudeFrame : uint8_t
{
  kRelativeToLaunch,
  kMeanSeaLevel,
};
}  // namespace tobas::mission

namespace YAML
{
template <>
struct convert<tobas::mission::AltitudeFrame>
{
  static Node encode(tobas::mission::AltitudeFrame _rhs);
  static bool decode(const Node& _node, tobas::mission::AltitudeFrame& _rhs);
};
}  // namespace YAML
