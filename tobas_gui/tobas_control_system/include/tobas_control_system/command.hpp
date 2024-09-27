#pragma once

namespace gui
{
namespace control_system
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
}  // namespace control_system
}  // namespace gui
