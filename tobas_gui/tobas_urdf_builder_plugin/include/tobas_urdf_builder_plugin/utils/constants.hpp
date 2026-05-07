// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

namespace tobas
{
namespace gui
{
namespace ub
{
static constexpr char kError[] = "ERROR";
static constexpr float kDefaultRobotAlpha = 0.7;  // FIXME: 1以外だとMeshのときに反映されない
static constexpr bool kDefaultVisualVisible = true;
static constexpr bool kDefaultCollisionVisible = true;
static constexpr bool kDefaultInertiaVisible = false;
}  // namespace ub
}  // namespace gui
}  // namespace tobas
