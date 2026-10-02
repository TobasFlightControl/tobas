// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <expected>

#include <QString>

namespace tobas
{
namespace gui
{
namespace cmn
{
/** Build a local project without blocking Qt’s main thread. */
std::expected<void, QString> buildLocalProject(const QString& proj_path);
}  // namespace cmn
}  // namespace gui
}  // namespace tobas
