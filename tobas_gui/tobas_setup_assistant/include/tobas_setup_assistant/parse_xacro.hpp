// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <expected>

#include <QString>

namespace tobas
{
namespace gui
{
namespace sa
{
std::expected<QString, QString> parseXacroFromPath(const QString& xacro_path);
}  // namespace sa
}  // namespace gui
}  // namespace tobas
