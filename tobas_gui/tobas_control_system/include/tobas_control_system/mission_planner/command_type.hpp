#pragma once

namespace gui
{
namespace ctrl
{
enum struct Command
{
  kWaypoint,
  kTakeoff,
  kLand,
  kReturnToLaunch,
};

const char* commandToText(Command cmd);
Command textToCommand(const char* text);
}  // namespace ctrl
}  // namespace gui
