// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QColor>
#include <QString>

namespace tobas
{
namespace qt
{
QString toCssColor(Qt::GlobalColor c, bool with_alpha = false);
}  // namespace qt
}  // namespace tobas
