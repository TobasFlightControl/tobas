// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QSize>
#include <QStackedWidget>

namespace tobas::qt
{
/**
 * ===== Differences from `QStackedWidget` =====
 * - Prefer the minimum height while retaining the preferred width
 * - Stabilize `setCurrentIndex`
 * - Additional methods
 */
class StackedWidget : public QStackedWidget
{
  Q_OBJECT

  using super = QStackedWidget;

public:
  using super::QStackedWidget;

  QSize sizeHint() const override;

  /** Delete all widgets and free memory. */
  void clear();

public Q_SLOTS:
  void setCurrentIndex(int _index);
};
}  // namespace tobas::qt
