// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QFont>

namespace tobas::qt {
/** `QFont` that uses the default style by default. */
class DefaultFont : public QFont {
public:
  explicit DefaultFont(int _point_size = -1,
                       QFont::Weight _weight = QFont::Weight::Normal,
                       bool _italic = false);
};
} // namespace tobas::qt
