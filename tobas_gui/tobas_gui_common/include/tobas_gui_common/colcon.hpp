// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QString>

#include <tobas_colcon_cpp/core.hpp>

namespace tobas
{
namespace gui
{
namespace cmn
{
/* Run `colcon build` without blocking Qt’s main thread. */
bool colconBuild(colcon::Colcon& colcon, const QString& pkg_path, const QString& ws_path);
}  // namespace cmn
}  // namespace gui
}  // namespace tobas
