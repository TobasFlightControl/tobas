// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_qt_tools/font.hpp"

namespace tobas
{
namespace qt
{
DefaultFont::DefaultFont(int _point_size, int _weight, bool _italic)
{
  if (_point_size > 0) {
    setPointSize(_point_size);
  }

  if (_weight >= 0) {
    setWeight(_weight);
  }

  setItalic(_italic);
}
}  // namespace qt
}  // namespace tobas
