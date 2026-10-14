// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

namespace tobas::gui::sim
{
bool waitUntilGazeboServerReady();
bool waitUntilGazeboRenderingReady();
void killGazeboServer();
}  // namespace tobas::gui::sim
