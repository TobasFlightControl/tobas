// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_qt_tools/widgets/check_box.hpp"

namespace tobas
{
namespace qt
{
CheckBox::CheckBox(const QString& text, bool checked, QWidget* parent) : super(text, parent)
{
  setChecked(checked);
}

void CheckBox::setDisabledTextNormal()
{
  auto pal = palette();
  const auto normal = pal.color(QPalette::Active, QPalette::WindowText);
  pal.setColor(QPalette::Disabled, QPalette::WindowText, normal);
  pal.setColor(QPalette::Disabled, QPalette::Text, normal);
  setPalette(pal);
}
}  // namespace qt
}  // namespace tobas
