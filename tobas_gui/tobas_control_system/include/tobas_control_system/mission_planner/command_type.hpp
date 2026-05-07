// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

namespace tobas
{
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
}  // namespace tobas
