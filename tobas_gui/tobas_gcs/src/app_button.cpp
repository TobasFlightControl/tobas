// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_gcs/app_button.hpp"

namespace tobas
{
namespace gui
{
namespace gcs
{
AppButton::AppButton(const QString& text, const QString& icon_path)
{
  setToolButtonStyle(Qt::ToolButtonTextUnderIcon);
  setText(text);
  setIcon(QIcon(icon_path));
  setIconSize(QSize(kButtonWidth, kIconHeight));  // Make the icon as wide as possible to keep the layout aligned.
  setFixedWidth(kButtonWidth);
  setCheckable(true);
}
}  // namespace gcs
}  // namespace gui
}  // namespace tobas
