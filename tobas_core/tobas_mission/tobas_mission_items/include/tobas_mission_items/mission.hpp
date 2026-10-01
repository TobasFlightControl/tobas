// SPDX-License-Identifier: Apache-2.0
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <cinttypes>
#include <vector>

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

struct Mission
{
  std::vector<MissionItem> items;
};
}  // namespace mission
}  // namespace tobas
