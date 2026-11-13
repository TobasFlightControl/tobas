// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QGridLayout>

namespace tobas::qt
{
/**
 * ===== Differences from `QFormLayout` =====
 * - Additional methods
 */
class GridLayout : public QGridLayout
{
  Q_OBJECT

public:
  using QGridLayout::QGridLayout;

  void clear();
};
}  // namespace tobas::qt
