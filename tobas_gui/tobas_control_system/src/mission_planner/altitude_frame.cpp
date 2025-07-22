#include "tobas_control_system/mission_planner/altitude_frame.hpp"

#include <string.h>

#include <format>
#include <stdexcept>

#define MEAN_SEA_LEVEL_LABEL "Mean Sea Level"
#define RELATIVE_TO_HOME_LABEL "Relative to Home"

namespace gui
{
namespace gcs
{
const char* altFrameToText(AltitudeFrame frame)
{
  switch (frame) {
    case AltitudeFrame::kMeanSeaLevel:
      return MEAN_SEA_LEVEL_LABEL;
    case AltitudeFrame::kRelativeToHome:
      return RELATIVE_TO_HOME_LABEL;
    default:
      throw std::runtime_error(std::format("Invalid altitude frame: {}", (int)frame));
  }
}

AltitudeFrame textToAltFrame(const char* text)
{
  if (strcmp(text, MEAN_SEA_LEVEL_LABEL) == 0) {
    return AltitudeFrame::kMeanSeaLevel;
  }
  else if (strcmp(text, RELATIVE_TO_HOME_LABEL) == 0) {
    return AltitudeFrame::kRelativeToHome;
  }
  else {
    throw std::runtime_error(std::format("Invalid altitude frame text: {}", text));
  }
}
}  // namespace gcs
}  // namespace gui
