// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control_system/mission_planner/command_type.hpp"

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
QString commandToText(mission::Type cmd)
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
      throw std::runtime_error("Invalid command type: " + std::to_string(cmd));
  }
}

mission::Type textToCommand(const QString& text)
{
  if (text == WAYPOINT_LABEL) {
    return mission::Type::kWaypoint;
  }
  else if (text == TAKEOFF_LABEL) {
    return mission::Type::kTakeoff;
  }
  else if (text == LAND_LABEL) {
    return mission::Type::kLand;
  }
  else if (text == RTL_LABEL) {
    return mission::Type::kReturnToLaunch;
  }
  else {
    throw std::runtime_error("Invalid command text: " + text.toStdString());
  }
}
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
