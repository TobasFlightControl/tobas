// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QFont>

namespace tobas::qt
{
/** Create a font with the specified style, retaining the default size when `_point_size` is not positive. */
QFont makeFont(int _point_size = 0, QFont::Weight _weight = QFont::Weight::Normal, bool _italic = false);
}  // namespace tobas::qt
