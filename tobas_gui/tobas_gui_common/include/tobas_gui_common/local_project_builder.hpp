// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <expected>

#include <QString>

namespace tobas::gui::cmn
{
/** Build a local project without blocking Qt’s main thread. */
std::expected<void, QString> buildLocalProject(const QString& _proj_path);
}  // namespace tobas::gui::cmn
