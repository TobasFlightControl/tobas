// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QListWidget>

namespace tobas::gui::sa
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
  void currentEntryChanged(int _id);

public:
  explicit SettingsNavigationWidget();

  /** Add a non-selectable section heading. */
  void addSection(const QString& _title);

  /** Add a selectable entry. IDs must be unique within the widget. */
  void addEntry(const QString& _title, int _id);

  /** Select the entry corresponding to `id`. */
  void setCurrentEntry(int _id);

  /** Enable or disable an entry, displaying `disabled_reason` as a tooltip while disabled. */
  void setEntryEnabled(int _id, bool _enabled, const QString& _disabled_reason = {});

protected:
  void currentChanged(const QModelIndex& _current, const QModelIndex& _previous) override;
  QModelIndex moveCursor(CursorAction _cursor_action, Qt::KeyboardModifiers _modifiers) override;
  void mousePressEvent(QMouseEvent* _event) override;
  bool viewportEvent(QEvent* _event) override;

private:
  bool restoring_current_item_ = false;

  QListWidgetItem* getEntry(int _id) const;
  void setListItemEnabled(QListWidgetItem* _item, bool _enabled);
  void onCurrentItemChanged(QListWidgetItem* _item);
};
}  // namespace tobas::gui::sa
