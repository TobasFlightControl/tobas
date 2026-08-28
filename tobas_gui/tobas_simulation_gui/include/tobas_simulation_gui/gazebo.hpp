// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

namespace tobas
{
namespace gui
{
namespace sim
{
bool waitUntilGazeboServerReady();
bool waitUntilGazeboRenderingReady();
void killGazeboServer();
}  // namespace sim
}  // namespace gui
}  // namespace tobas
