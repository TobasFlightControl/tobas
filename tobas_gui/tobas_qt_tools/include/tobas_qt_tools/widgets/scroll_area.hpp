// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QScrollArea>

namespace tobas::qt
{
/**
 * ===== Differences from `QScrollArea` =====
 * - Scrollable by default
 * - Additional methods
 */
class ScrollArea : public QScrollArea
{
  Q_OBJECT

public:
  explicit ScrollArea(QWidget* _parent = nullptr);

  /** Set a layout inside the widget. */
  void setLayout(QLayout* _layout);

  /** Make the background transparent. */
  void setBackgroundTransparent();
};
}  // namespace tobas::qt
