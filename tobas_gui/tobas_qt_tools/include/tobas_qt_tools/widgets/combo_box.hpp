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

  void wheelEvent(QWheelEvent* _event) override;

  bool contains(const QString& _text) const;

  void removeText(const QString& _text);

  void setItemEnabled(int _row, bool _enabled);
  void setItemEnabled(const QString& _text, bool _enabled);

  void sort();

public Q_SLOTS:
  void setCurrentIndex(int _index);
  void setCurrentText(const QString& _text);
};
}  // namespace qt
}  // namespace tobas
