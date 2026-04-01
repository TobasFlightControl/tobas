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
const char* commandToText(Command cmd)
{
  switch (cmd) {
    case Command::kWaypoint:
      return WAYPOINT_LABEL;
    case Command::kTakeoff:
      return TAKEOFF_LABEL;
    case Command::kLand:
      return LAND_LABEL;
    case Command::kReturnToLaunch:
      return RTL_LABEL;
    default:
      throw std::runtime_error(std::format("Invalid command type: {}", (int)cmd));
  }
}

Command textToCommand(const char* text)
{
  if (strcmp(text, WAYPOINT_LABEL) == 0) {
    return Command::kWaypoint;
  }
  else if (strcmp(text, TAKEOFF_LABEL) == 0) {
    return Command::kTakeoff;
  }
  else if (strcmp(text, LAND_LABEL) == 0) {
    return Command::kLand;
  }
  else if (strcmp(text, RTL_LABEL) == 0) {
    return Command::kReturnToLaunch;
  }
  else {
    throw std::runtime_error(std::format("Invalid command text: {}", text));
  }
}
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
