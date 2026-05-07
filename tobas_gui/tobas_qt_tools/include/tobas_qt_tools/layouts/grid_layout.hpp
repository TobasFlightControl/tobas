// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QGridLayout>

namespace tobas
{
namespace qt
{
/**
 * ===== QFormLayout との違い =====
 * - 追加メソッド
 */
class GridLayout : public QGridLayout
{
  Q_OBJECT

  using self = GridLayout;
  using super = QGridLayout;

public:
  using QGridLayout::QGridLayout;

  void clear();
};
}  // namespace qt
}  // namespace tobas
