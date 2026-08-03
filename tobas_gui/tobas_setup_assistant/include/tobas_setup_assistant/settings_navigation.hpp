// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QListWidget>

namespace tobas
{
namespace gui
{
namespace sa
{
class SettingsNavigationWidget : public QListWidget
{
  Q_OBJECT

  using self = SettingsNavigationWidget;
  using super = QListWidget;

Q_SIGNALS:
  void currentEntryChanged(int id);

public:
  explicit SettingsNavigationWidget();

  void addSection(const QString& title);
  void addEntry(const QString& title, int id);
  void setCurrentEntry(int id);
  void setEntryEnabled(int id, bool enabled);

protected:
  void currentChanged(const QModelIndex& current, const QModelIndex& previous) override;
  QModelIndex moveCursor(CursorAction cursor_action, Qt::KeyboardModifiers modifiers) override;
  void mousePressEvent(QMouseEvent* event) override;

private:
  bool restoring_current_item_ = false;

  QListWidgetItem* getEntry(int id) const;
  void setListItemEnabled(QListWidgetItem* item, bool enabled);
  void onCurrentItemChanged(QListWidgetItem* item);
};
}  // namespace sa
}  // namespace gui
}  // namespace tobas
