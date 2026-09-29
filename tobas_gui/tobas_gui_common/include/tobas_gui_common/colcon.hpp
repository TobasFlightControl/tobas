// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <expected>

#include <QString>

#include <tobas_colcon_cpp/core.hpp>

namespace tobas
{
namespace gui
{
namespace cmn
{
/* Run `colcon build` without blocking Qt’s main thread. */
std::expected<void, QString>
colconBuild(const QString& pkg_path, const QString& ws_path, const colcon::BuildOptions& options = {});
}  // namespace cmn
}  // namespace gui
}  // namespace tobas
