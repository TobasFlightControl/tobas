#include "tobas_control_system/mission_planner/command.hpp"

#include <string.h>

#include <format>
#include <stdexcept>

#define WAYPOINT_LABEL "Waypoint"
#define TAKEOFF_LABEL "Takeoff"
#define LAND_LABEL "Land"
#define RETURN_TO_HOME_LABEL "Return to Home"

namespace gui
{
namespace gcs
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
    case Command::kReturnToHome:
      return RETURN_TO_HOME_LABEL;
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
  else if (strcmp(text, RETURN_TO_HOME_LABEL) == 0) {
    return Command::kReturnToHome;
  }
  else {
    throw std::runtime_error(std::format("Invalid command text: {}", text));
  }
}
}  // namespace gcs
}  // namespace gui
