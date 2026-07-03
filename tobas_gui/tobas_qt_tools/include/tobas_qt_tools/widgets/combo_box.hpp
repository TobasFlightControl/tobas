// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QComboBox>

namespace tobas
{
namespace qt
{
/**
 * ===== Differences from `QComboBox` =====
 * - Disable mouse wheel events
 * - Range check in `setCurrentIndex`
 * - Error when `setCurrentText` specifies a nonexistent option
 * - Additional methods
 */
class ComboBox : public QComboBox
{
  Q_OBJECT

  using super = QComboBox;

public:
  using super::QComboBox;

  void wheelEvent(QWheelEvent* event) override;

  bool contains(const QString& text) const;

  void removeText(const QString& text);

  void setItemEnabled(int row, bool enabled);
  void setItemEnabled(const QString& text, bool enabled);

  void sort();

public Q_SLOTS:
  void setCurrentIndex(int index);
  void setCurrentText(const QString& text);
};
}  // namespace qt
}  // namespace tobas
