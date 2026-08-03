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
/**
 * Grouped, single-selection navigation list.
 *
 * Sections are non-selectable headings. Entries are identified by caller-provided integer IDs, allowing this widget
 * to remain independent of the content widget that reacts to a selection.
 */
class SettingsNavigationWidget : public QListWidget
{
  Q_OBJECT

  using self = SettingsNavigationWidget;
  using super = QListWidget;

Q_SIGNALS:
  /** Emitted when the user or caller selects an entry. */
  void currentEntryChanged(int id);

public:
  explicit SettingsNavigationWidget();

  /** Add a non-selectable section heading. */
  void addSection(const QString& title);

  /** Add a selectable entry. IDs must be unique within the widget. */
  void addEntry(const QString& title, int id);

  /** Select the entry corresponding to `id`. */
  void setCurrentEntry(int id);

  /** Enable or disable both interaction and selection for the entry corresponding to `id`. */
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
