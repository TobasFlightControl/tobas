// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_gcs/app_button.hpp"

namespace tobas::gui::gcs
{
AppButton::AppButton(const QString& _text, const QString& _icon_path)
{
  setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
  setText(_text);
  setIcon(QIcon(_icon_path));

  constexpr int kButtonWidth = 120;
  setIconSize(QSize(kButtonWidth, 40));  // Make the icon as wide as possible to keep the layout aligned.
  setFixedWidth(kButtonWidth);

  setCheckable(true);
}
}  // namespace tobas::gui::gcs
