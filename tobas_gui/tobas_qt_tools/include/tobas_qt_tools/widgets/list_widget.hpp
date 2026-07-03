// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QListWidget>

namespace tobas
{
namespace qt
{
/**
 * ===== Differences from `QListWidgetItem` =====
 * - Emit signals on drag and drop
 * - Additional methods
 */
class ListWidget : public QListWidget
{
  Q_OBJECT

  using super = QListWidget;

Q_SIGNALS:
  void itemMoved(QListWidgetItem* item);

public:
  using super::QListWidget;

  /* Return true if the list contains the text. */
  bool contains(const QString& text) const;

  /* Delete an item. */
  void remove(QListWidgetItem* item);

  /* Select the item with the specified text. */
  void setCurrentText(const QString& text);

  /* Clear the selection. */
  void deselect();

  /* Set the list height to the number of rows. */
  void shrinkToContents();

  /* Show row numbers. */
  void showRowNumber();

protected:
  void dropEvent(QDropEvent* event) override;
};

/**
 * ===== Differences from `QListWidgetItem` =====
 * - Compare by `UserRole`
 */
class ListWidgetItem : public QListWidgetItem
{
public:
  virtual bool operator<(const QListWidgetItem& rhs) const override;
};
}  // namespace qt
}  // namespace tobas
