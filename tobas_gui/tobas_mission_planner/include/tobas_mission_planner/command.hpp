#pragma once

namespace gui
{
namespace mission_planner
{
enum command_t
{
  WAYPOINT,
  TAKEOFF,
  LAND,
  RETURN_TO_HOME,
};

const char* commandToText(command_t cmd);
command_t textToCommand(const char* text);
}  // namespace mission_planner
}  // namespace gui
