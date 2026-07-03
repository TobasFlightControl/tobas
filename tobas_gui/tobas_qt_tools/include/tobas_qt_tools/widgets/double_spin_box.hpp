// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QDoubleSpinBox>

namespace tobas
{
namespace qt
{
/**
 * ===== Differences from `QDoubleSpinBox` =====
 * - Set default minimum and maximum values to the minimum and maximum of `double`.
 * - Disable mouse wheel events
 * - Select all text on focus
 */
class DoubleSpinBox : public QDoubleSpinBox
{
  Q_OBJECT

  using super = QDoubleSpinBox;

public:
  explicit DoubleSpinBox(QWidget* parent = nullptr);

protected:
  void wheelEvent(QWheelEvent* event) override;
  void focusInEvent(QFocusEvent* event) override;
};
}  // namespace qt
}  // namespace tobas
