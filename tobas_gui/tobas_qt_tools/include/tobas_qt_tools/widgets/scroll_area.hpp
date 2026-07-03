// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QScrollArea>

namespace tobas
{
namespace qt
{
/**
 * ===== Differences from `QScrollArea` =====
 * - Scrollable by default
 * - Additional methods
 */
class ScrollArea : public QScrollArea
{
  Q_OBJECT

  using super = QScrollArea;

public:
  explicit ScrollArea(QWidget* parent = nullptr);

  /* Set a layout inside the widget. */
  void setLayout(QLayout* layout);

  /* Make the background transparent. */
  void setBackgroundTransparent();
};
}  // namespace qt
}  // namespace tobas
