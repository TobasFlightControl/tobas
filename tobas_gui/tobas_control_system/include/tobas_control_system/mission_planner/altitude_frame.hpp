#pragma once

#include <cstdint>

namespace gui
{
namespace ctrl
{
enum AltitudeFrame
{
  kMeanSeaLevel,
  kRelativeToHome,
};

const char* altFrameToText(AltitudeFrame frame);
AltitudeFrame textToAltFrame(const char* text);
}  // namespace ctrl
}  // namespace gui
