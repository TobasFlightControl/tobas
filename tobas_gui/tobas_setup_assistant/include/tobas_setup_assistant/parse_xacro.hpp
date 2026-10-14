// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <expected>

#include <QString>

namespace tobas::gui::sa
{
std::expected<QString, QString> parseXacroFromPath(const QString& _xacro_path);
}  // namespace tobas::gui::sa
