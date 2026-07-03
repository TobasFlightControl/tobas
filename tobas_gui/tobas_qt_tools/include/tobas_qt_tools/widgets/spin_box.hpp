// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QSpinBox>

namespace tobas
{
namespace qt
{
/**
 * ===== Differences from `QSpinBox` =====
 * - Set default minimum and maximum values to the minimum and maximum of `int32`.
 * - Disable mouse wheel events
 * - Select all text on focus
 */
class SpinBox : public QSpinBox
{
  Q_OBJECT

  using super = QSpinBox;

public:
  explicit SpinBox(QWidget* parent = nullptr);

protected:
  void wheelEvent(QWheelEvent* event) override;
  void focusInEvent(QFocusEvent* event) override;
};
}  // namespace qt
}  // namespace tobas
