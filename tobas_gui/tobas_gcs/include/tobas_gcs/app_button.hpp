// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QToolButton>

namespace tobas
{
namespace gui
{
namespace gcs
{
class AppButton : public QToolButton
{
  Q_OBJECT

  using self = AppButton;
  using super = QToolButton;

  static constexpr int kIconHeight = 40;
  static constexpr int kButtonWidth = 120;

public:
  explicit AppButton(const QString& text, const QString& icon_path);
};
}  // namespace gcs
}  // namespace gui
}  // namespace tobas
