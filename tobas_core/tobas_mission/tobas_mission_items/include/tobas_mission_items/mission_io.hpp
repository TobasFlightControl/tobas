// SPDX-License-Identifier: Apache-2.0
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <expected>
#include <string>

#include <yaml-cpp/yaml.h>

#include "tobas_mission_items/mission.hpp"

namespace tobas
{
namespace mission
{
YAML::Node dumpMission(const Mission& mission);
std::expected<Mission, std::string> loadMission(const YAML::Node& node);
}  // namespace mission
}  // namespace tobas
