// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QLabel>

namespace tobas::qt
{
/**
 * ===== Differences from `QLabel` =====
 * - Font can be specified in the constructor
 * - Additional methods
 */
class Label : public QLabel
{
  using super = QLabel;

public:
  explicit Label(
    const QString& _text = "",
    int _point_size = -1,
    QFont::Weight _weight = QFont::Weight::Normal,
    bool _italic = false,
    QWidget* _parent = nullptr);

  void setTextColor(const QString& _color);
  void setTextColor(Qt::GlobalColor _color);
};
}  // namespace tobas::qt
