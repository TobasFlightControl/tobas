// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QCheckBox>

namespace tobas::qt
{
/**
 * ===== Differences from `QCheckBox` =====
 * - Constructor with initial state as an argument
 * - Additional methods
 */
class CheckBox : public QCheckBox
{
  Q_OBJECT

  using super = QCheckBox;

public:
  using super::QCheckBox;

  explicit CheckBox(const QString& _text, bool _checked, QWidget* _parent = nullptr);

  /** Display only the text normally even when disabled. */
  void setDisabledTextNormal();
};
}  // namespace tobas::qt
