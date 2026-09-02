// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control_system/mission_planner/command_type.hpp"

#include <stdexcept>

namespace tobas
{
namespace gui
{
namespace ctrl
{
namespace
{
constexpr char kWaypointLabel[] = "Waypoint";
constexpr char kTakeoffLabel[] = "Takeoff";
constexpr char kLandLabel[] = "Land";
constexpr char kRtlLabel[] = "Return to Launch";
}  // namespace

QString commandToText(mission::Type cmd)
{
  switch (cmd) {
    case mission::Type::kWaypoint:
      return kWaypointLabel;
    case mission::Type::kTakeoff:
      return kTakeoffLabel;
    case mission::Type::kLand:
      return kLandLabel;
    case mission::Type::kReturnToLaunch:
      return kRtlLabel;
    default:
      throw std::runtime_error("Invalid command type: " + std::to_string(cmd));
  }
}

mission::Type textToCommand(const QString& text)
{
  if (text == kWaypointLabel) {
    return mission::Type::kWaypoint;
  }
  else if (text == kTakeoffLabel) {
    return mission::Type::kTakeoff;
  }
  else if (text == kLandLabel) {
    return mission::Type::kLand;
  }
  else if (text == kRtlLabel) {
    return mission::Type::kReturnToLaunch;
  }
  else {
    throw std::runtime_error("Invalid command text: " + text.toStdString());
  }
}
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
