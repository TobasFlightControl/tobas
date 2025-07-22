#pragma once

#include <cstdint>

namespace gui
{
namespace gcs
{
enum AltitudeFrame
{
  kMeanSeaLevel,
  kRelativeToHome,
};

const char* altFrameToText(AltitudeFrame frame);
AltitudeFrame textToAltFrame(const char* text);
}  // namespace gcs
}  // namespace gui
