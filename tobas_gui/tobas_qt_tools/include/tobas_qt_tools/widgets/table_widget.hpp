// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QTableWidget>

namespace tobas
{
namespace qt
{
/**
 * ===== Differences from `TableWidget` =====
 * - Additional methods
 */
class TableWidget : public QTableWidget
{
  Q_OBJECT

  using super = QTableWidget;

public:
  using super::QTableWidget;

  /* Delete all rows. Unlike `clear`, this deletes cells as well as content. */
  void removeAll();

  /* Fix all column widths uniformly. */
  void setColumnsWidth(int width);

  /* Adjust table height to fit content. */
  void resizeHeightToContents();

  /* Set clickability in bulk. */
  void setHeaderSectionsClickable(bool clickable);
};
}  // namespace qt
}  // namespace tobas
