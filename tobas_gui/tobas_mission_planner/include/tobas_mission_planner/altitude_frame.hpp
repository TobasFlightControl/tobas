#pragma once

#include <cstdint>

namespace gui
{
namespace mission_planner
{
enum altitude_frame_t : uint8_t
{
  MEAN_SEA_LEVEL,
  RELATIVE_TO_HOME,
};

const char* altFrameToText(altitude_frame_t frame);
altitude_frame_t textToAltFrame(const char* text);
}  // namespace mission_planner
}  // namespace gui
