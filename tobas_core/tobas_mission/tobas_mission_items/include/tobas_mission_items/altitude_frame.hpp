// SPDX-License-Identifier: Apache-2.0
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <yaml-cpp/yaml.h>

namespace tobas
{
namespace mission
{
enum AltitudeFrame : uint8_t
{
  kRelativeToLaunch,
  kMeanSeaLevel,
};
}  // namespace mission
}  // namespace tobas

namespace YAML
{
template <>
struct convert<tobas::mission::AltitudeFrame>
{
  static Node encode(const tobas::mission::AltitudeFrame& rhs);
  static bool decode(const Node& node, tobas::mission::AltitudeFrame& rhs);
};
}  // namespace YAML
