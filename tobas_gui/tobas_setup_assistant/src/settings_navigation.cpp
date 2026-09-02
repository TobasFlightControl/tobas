// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/settings_navigation.hpp"

#include <stdexcept>

#include <QHelpEvent>
#include <QItemSelectionModel>
#include <QMouseEvent>
#include <QToolTip>

namespace tobas
{
namespace gui
{
namespace sa
{
namespace
{
bool isSelectableEntry(const QModelIndex& index)
{
  // Entries store their IDs in UserRole. Section headings and spacers deliberately leave this role empty.
  constexpr auto kRequiredFlags = Qt::ItemIsEnabled | Qt::ItemIsSelectable;
  return index.isValid() && index.data(Qt::UserRole).isValid() &&
         (index.model()->flags(index) & kRequiredFlags) == kRequiredFlags;
}
}  // namespace

SettingsNavigationWidget::SettingsNavigationWidget()
{
  setObjectName("settingsNavigation");
  setSelectionMode(QAbstractItemView::SingleSelection);
  setStyleSheet(R"(
    QListWidget#settingsNavigation {
      background-color: palette(base);
      border: 1px solid palette(midlight);
      outline: none;
      padding: 6px 0;
    }
    QListWidget#settingsNavigation::item {
      border: none;
      padding: 0 12px;
    }
    QListWidget#settingsNavigation::item:selected {
      background-color: palette(highlight);
      color: palette(highlighted-text);
    }
  )");

  connect(this, &QListWidget::currentItemChanged, this, &self::onCurrentItemChanged);
}

void SettingsNavigationWidget::addSection(const QString& title)
{
  // The list's own top padding covers the first section; only subsequent sections need additional separation.
  if (count() > 0) {
    const auto spacer = new QListWidgetItem(this);
    spacer->setFlags(Qt::NoItemFlags);
    spacer->setSizeHint(QSize(0, 12));
  }

  const auto item = new QListWidgetItem(title, this);
  item->setFlags(Qt::ItemIsEnabled);  // Keep the heading visually enabled while excluding it from selection.
  item->setSizeHint(QSize(0, 28));

  auto font = item->font();
  font.setBold(true);
  item->setFont(font);
}

void SettingsNavigationWidget::addEntry(const QString& title, int id)
{
  const auto item = new QListWidgetItem(title, this);
  item->setData(Qt::UserRole, id);
  item->setSizeHint(QSize(0, 38));
}

void SettingsNavigationWidget::setCurrentEntry(int id)
{
  setCurrentItem(getEntry(id));
}

void SettingsNavigationWidget::setEntryEnabled(int id, bool enabled, const QString& disabled_reason)
{
  const auto item = getEntry(id);
  item->setToolTip(enabled ? "" : disabled_reason);
  setListItemEnabled(item, enabled);
}

void SettingsNavigationWidget::currentChanged(const QModelIndex& current, const QModelIndex& previous)
{
  const auto is_non_entry = current.isValid() && !current.data(Qt::UserRole).isValid();
  if (!restoring_current_item_ && is_non_entry) {
    // Reject current-index changes to headings and spacers, including changes made outside mouse handling.
    restoring_current_item_ = true;
    if (previous.isValid()) {
      selectionModel()->setCurrentIndex(previous, QItemSelectionModel::ClearAndSelect);
    }
    else {
      selectionModel()->clearCurrentIndex();
    }
    restoring_current_item_ = false;
    return;
  }

  super::currentChanged(current, previous);
}

QModelIndex SettingsNavigationWidget::moveCursor(CursorAction cursor_action, Qt::KeyboardModifiers modifiers)
{
  const auto next = super::moveCursor(cursor_action, modifiers);
  if (isSelectableEntry(next)) {
    return next;
  }

  // Continue in the requested direction when Qt's default movement lands on a heading, spacer, or disabled entry.
  int direction = 0;
  switch (cursor_action) {
    case MoveUp:
    case MoveLeft:
    case MovePageUp:
    case MovePrevious:
    case MoveEnd:
      direction = -1;
      break;
    case MoveDown:
    case MoveRight:
    case MovePageDown:
    case MoveNext:
    case MoveHome:
      direction = 1;
      break;
    default:
      throw std::runtime_error("Invalid cursor action.");
  }

  for (auto row = next.row() + direction; row >= 0 && row < model()->rowCount(); row += direction) {
    const auto candidate = model()->index(row, 0);
    if (isSelectableEntry(candidate)) {
      return candidate;
    }
  }

  return currentIndex();
}

void SettingsNavigationWidget::mousePressEvent(QMouseEvent* event)
{
  const auto item = itemAt(event->pos());
  if (item && !item->data(Qt::UserRole).isValid()) {
    // Preserve the current entry when a heading or spacer is clicked.
    event->accept();
    return;
  }

  super::mousePressEvent(event);
}

bool SettingsNavigationWidget::viewportEvent(QEvent* event)
{
  if (event->type() == QEvent::ToolTip) {
    const auto help_event = static_cast<QHelpEvent*>(event);
    const auto item = itemAt(help_event->pos());
    const auto is_disabled = item && !item->flags().testFlag(Qt::ItemIsEnabled);
    if (is_disabled && !item->toolTip().isEmpty()) {
      // Disabled items do not reliably receive Qt's standard tooltip handling, so display their reason explicitly.
      QToolTip::showText(help_event->globalPos(), item->toolTip(), viewport(), visualItemRect(item));
      return true;
    }
  }

  return super::viewportEvent(event);
}

QListWidgetItem* SettingsNavigationWidget::getEntry(int id) const
{
  for (int row = 0; row < count(); ++row) {
    const auto item = this->item(row);
    if (item->data(Qt::UserRole) == id) {
      return item;
    }
  }

  throw std::runtime_error("Navigation entry not found.");
}

void SettingsNavigationWidget::setListItemEnabled(QListWidgetItem* item, bool enabled)
{
  constexpr auto kEnableFlags = Qt::ItemIsSelectable | Qt::ItemIsEnabled;

  if (enabled) {
    item->setFlags(item->flags() | kEnableFlags);
  }
  else {
    item->setFlags(item->flags() & ~kEnableFlags);
  }
}

void SettingsNavigationWidget::onCurrentItemChanged(QListWidgetItem* item)
{
  if (!item) {
    return;
  }

  const auto id = item->data(Qt::UserRole);
  if (id.isValid()) {
    Q_EMIT currentEntryChanged(id.toInt());
  }
}
}  // namespace sa
}  // namespace gui
}  // namespace tobas
