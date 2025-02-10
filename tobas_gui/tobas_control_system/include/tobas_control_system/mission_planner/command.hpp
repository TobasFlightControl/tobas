#pragma once

namespace gui
{
namespace gcs
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
}  // namespace gcs
}  // namespace gui
