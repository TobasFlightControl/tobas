// SPDX-License-Identifier: Apache-2.0
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <cinttypes>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "./mission_items.hpp"

namespace tobas
{
namespace mission
{
enum Type : uint8_t
{
  kWaypoint,
  kTakeoff,
  kLand,
  kReturnToLaunch,
};

struct MissionItem
{
  Type type;
  std::vector<uint8_t> data;
};

class Mission
{
public:
  std::vector<MissionItem> items;

  YAML::Node dump() const;
  bool load(const YAML::Node& node);
};
}  // namespace mission
}  // namespace tobas
