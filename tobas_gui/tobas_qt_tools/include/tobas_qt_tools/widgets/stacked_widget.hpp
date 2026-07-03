// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QStackedWidget>

namespace tobas
{
namespace qt
{
/**
 * ===== Differences from `QStackedWidget` =====
 * - Stabilize `setCurrentIndex`
 * - Additional methods
 */
class StackedWidget : public QStackedWidget
{
  Q_OBJECT

  using super = QStackedWidget;

public:
  using super::QStackedWidget;

  /* Delete all widgets and free memory. */
  void clear();

public Q_SLOTS:
  void setCurrentIndex(int index);
};
}  // namespace qt
}  // namespace tobas
