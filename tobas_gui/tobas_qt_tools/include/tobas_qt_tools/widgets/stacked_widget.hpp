// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QStackedWidget>

namespace tobas
{
namespace qt
{
/**
 * ===== QStackedWidget との違い =====
 * - setCurrentIndexを安定化
 * - 追加メソッド
 */
class StackedWidget : public QStackedWidget
{
  Q_OBJECT

  using super = QStackedWidget;

public:
  using super::QStackedWidget;

  /* 全てのウィジェットを削除し，メモリを開放する． */
  void clear();

public Q_SLOTS:
  void setCurrentIndex(int index);
};
}  // namespace qt
}  // namespace tobas
