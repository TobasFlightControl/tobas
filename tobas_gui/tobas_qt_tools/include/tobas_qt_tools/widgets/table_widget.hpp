// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QTableWidget>

namespace tobas
{
namespace qt
{
/**
 * ===== TableWidget との違い =====
 * - 追加メソッド
 */
class TableWidget : public QTableWidget
{
  Q_OBJECT

  using super = QTableWidget;

public:
  using super::QTableWidget;

  /* 全ての行を削除する．clearとは異なり，内容に加えセルまで削除する． */
  void removeAll();

  /* 全ての列幅を一様に固定する． */
  void setColumnsWidth(int width);

  /* 内容に合わせてテーブルの高さを調整する． */
  void resizeHeightToContents();

  /* クリック可否を一括で設定する． */
  void setHeaderSectionsClickable(bool clickable);
};
}  // namespace qt
}  // namespace tobas
