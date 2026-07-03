// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QLabel>

namespace tobas
{
namespace qt
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
    const QString& text = "",
    int point_size = -1,
    int weight = -1,
    bool italic = false,
    QWidget* parent = nullptr);

  void setTextColor(const QString color);
  void setTextColor(const Qt::GlobalColor color);
};
}  // namespace qt
}  // namespace tobas
