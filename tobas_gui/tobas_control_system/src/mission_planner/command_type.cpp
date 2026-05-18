// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control_system/mission_planner/command_type.hpp"

#include <string.h>

#include <format>
#include <stdexcept>

#define WAYPOINT_LABEL "Waypoint"
#define TAKEOFF_LABEL "Takeoff"
#define LAND_LABEL "Land"
#define RTL_LABEL "Return to Launch"

namespace tobas
{
namespace gui
{
namespace ctrl
{
const char* commandToText(mission::Type cmd)
{
  switch (cmd) {
    case mission::Type::kWaypoint:
      return WAYPOINT_LABEL;
    case mission::Type::kTakeoff:
      return TAKEOFF_LABEL;
    case mission::Type::kLand:
      return LAND_LABEL;
    case mission::Type::kReturnToLaunch:
      return RTL_LABEL;
    default:
      throw std::runtime_error(std::format("Invalid command type: {}", (int)cmd));
  }
}

mission::Type textToCommand(const char* text)
{
  if (strcmp(text, WAYPOINT_LABEL) == 0) {
    return mission::Type::kWaypoint;
  }
  else if (strcmp(text, TAKEOFF_LABEL) == 0) {
    return mission::Type::kTakeoff;
  }
  else if (strcmp(text, LAND_LABEL) == 0) {
    return mission::Type::kLand;
  }
  else if (strcmp(text, RTL_LABEL) == 0) {
    return mission::Type::kReturnToLaunch;
  }
  else {
    throw std::runtime_error(std::format("Invalid command text: {}", text));
  }
}
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
