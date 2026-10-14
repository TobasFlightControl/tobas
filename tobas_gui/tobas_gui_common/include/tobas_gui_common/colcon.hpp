// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <expected>

#include <QString>

#include <tobas_colcon_cpp/core.hpp>

namespace tobas::gui::cmn
{
/** Run `colcon build` without blocking Qt’s main thread. */
std::expected<void, QString>
colconBuild(const QString& _pkg_path, const QString& _ws_path, const colcon::BuildOptions& _options = {});
}  // namespace tobas::gui::cmn
