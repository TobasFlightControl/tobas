// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_qt_tools/font.hpp"

namespace tobas::qt
{
QFont makeFont(int _point_size, QFont::Weight _weight, bool _italic)
{
  QFont font;
  if (_point_size > 0) {
    font.setPointSize(_point_size);
  }

  if (_weight >= 0) {
    font.setWeight(_weight);
  }

  font.setItalic(_italic);

  return font;
}
}  // namespace tobas::qt
