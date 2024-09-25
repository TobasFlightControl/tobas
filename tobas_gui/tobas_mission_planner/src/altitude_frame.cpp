#include <stdexcept>
#include <format>
#include <string.h>

#include "tobas_mission_planner/altitude_frame.hpp"

#define MEAN_SEA_LEVEL_LABEL "Mean Sea Level"
#define RELATIVE_TO_HOME_LABEL "Relative to Home"

namespace gui
{
namespace mission_planner
{
const char* altFrameToText(altitude_frame_t frame)
{
  switch (frame)
  {
    case altitude_frame_t::MEAN_SEA_LEVEL:
      return MEAN_SEA_LEVEL_LABEL;
    case altitude_frame_t::RELATIVE_TO_HOME:
      return RELATIVE_TO_HOME_LABEL;
    default:
      throw std::runtime_error(std::format("Invalid altitude frame: {}", (int)frame));
  }
}

altitude_frame_t textToAltFrame(const char* text)
{
  if (strcmp(text, MEAN_SEA_LEVEL_LABEL) == 0)
    return altitude_frame_t::MEAN_SEA_LEVEL;
  else if (strcmp(text, RELATIVE_TO_HOME_LABEL) == 0)
    return altitude_frame_t::RELATIVE_TO_HOME;
  else
    throw std::runtime_error(std::format("Invalid altitude frame text: {}", text));
}
}  // namespace mission_planner
}  // namespace gui
