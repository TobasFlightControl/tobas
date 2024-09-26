#include <stdexcept>
#include <format>
#include <string.h>

#include "tobas_mission_planner/command.hpp"

#define WAYPOINT_LABEL "Waypoint"
#define TAKEOFF_LABEL "Takeoff"
#define LAND_LABEL "Land"
#define RETURN_TO_HOME_LABEL "Return to Home"

namespace gui
{
namespace mission_planner
{
const char* commandToText(command_t cmd)
{
  switch (cmd)
  {
    case command_t::WAYPOINT:
      return WAYPOINT_LABEL;
    case command_t::TAKEOFF:
      return TAKEOFF_LABEL;
    case command_t::LAND:
      return LAND_LABEL;
    case command_t::RETURN_TO_HOME:
      return RETURN_TO_HOME_LABEL;
    default:
      throw std::runtime_error(std::format("Invalid command type: {}", (int)cmd));
  }
}

command_t textToCommand(const char* text)
{
  if (strcmp(text, WAYPOINT_LABEL) == 0)
    return command_t::WAYPOINT;
  else if (strcmp(text, TAKEOFF_LABEL) == 0)
    return command_t::TAKEOFF;
  else if (strcmp(text, LAND_LABEL) == 0)
    return command_t::LAND;
  else if (strcmp(text, RETURN_TO_HOME_LABEL) == 0)
    return command_t::RETURN_TO_HOME;
  else
    throw std::runtime_error(std::format("Invalid command text: {}", text));
}
}  // namespace mission_planner
}  // namespace gui
