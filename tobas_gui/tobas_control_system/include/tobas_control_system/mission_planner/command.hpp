#pragma once

namespace gui
{
namespace gcs
{
enum struct Command
{
  kWaypoint,
  kTakeoff,
  kLand,
  kReturnToHome,
};

const char* commandToText(Command cmd);
Command textToCommand(const char* text);
}  // namespace gcs
}  // namespace gui
