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
bool isSelectableEntry(const QModelIndex& _index)
{
  // Entries store their IDs in UserRole. Section headings and spacers deliberately leave this role empty.
  constexpr auto kRequiredFlags = Qt::ItemIsEnabled | Qt::ItemIsSelectable;
  return _index.isValid() && _index.data(Qt::UserRole).isValid() &&
         (_index.model()->flags(_index) & kRequiredFlags) == kRequiredFlags;
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

void SettingsNavigationWidget::addSection(const QString& _title)
{
  // The list's own top padding covers the first section; only subsequent sections need additional separation.
  if (count() > 0) {
    const auto spacer = new QListWidgetItem(this);
    spacer->setFlags(Qt::NoItemFlags);
    spacer->setSizeHint(QSize(0, 12));
  }

  const auto item = new QListWidgetItem(_title, this);
  item->setFlags(Qt::ItemIsEnabled);  // Keep the heading visually enabled while excluding it from selection.
  item->setSizeHint(QSize(0, 28));

  auto font = item->font();
  font.setBold(true);
  item->setFont(font);
}

void SettingsNavigationWidget::addEntry(const QString& _title, int _id)
{
  const auto item = new QListWidgetItem(_title, this);
  item->setData(Qt::UserRole, _id);
  item->setSizeHint(QSize(0, 38));
}

void SettingsNavigationWidget::setCurrentEntry(int _id)
{
  setCurrentItem(getEntry(_id));
}

void SettingsNavigationWidget::setEntryEnabled(int _id, bool _enabled, const QString& _disabled_reason)
{
  const auto item = getEntry(_id);
  item->setToolTip(_enabled ? "" : _disabled_reason);
  setListItemEnabled(item, _enabled);
}

void SettingsNavigationWidget::currentChanged(const QModelIndex& _current, const QModelIndex& _previous)
{
  const auto is_non_entry = _current.isValid() && !_current.data(Qt::UserRole).isValid();
  if (!restoring_current_item_ && is_non_entry) {
    // Reject current-index changes to headings and spacers, including changes made outside mouse handling.
    restoring_current_item_ = true;
    if (_previous.isValid()) {
      selectionModel()->setCurrentIndex(_previous, QItemSelectionModel::ClearAndSelect);
    }
    else {
      selectionModel()->clearCurrentIndex();
    }
    restoring_current_item_ = false;
    return;
  }

  super::currentChanged(_current, _previous);
}

QModelIndex SettingsNavigationWidget::moveCursor(CursorAction _cursor_action, Qt::KeyboardModifiers _modifiers)
{
  const auto next = super::moveCursor(_cursor_action, _modifiers);
  if (isSelectableEntry(next)) {
    return next;
  }

  // Continue in the requested direction when Qt's default movement lands on a heading, spacer, or disabled entry.
  int direction = 0;
  switch (_cursor_action) {
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

void SettingsNavigationWidget::mousePressEvent(QMouseEvent* _event)
{
  const auto item = itemAt(_event->pos());
  if (item && !item->data(Qt::UserRole).isValid()) {
    // Preserve the current entry when a heading or spacer is clicked.
    _event->accept();
    return;
  }

  super::mousePressEvent(_event);
}

bool SettingsNavigationWidget::viewportEvent(QEvent* _event)
{
  if (_event->type() == QEvent::ToolTip) {
    const auto help_event = static_cast<QHelpEvent*>(_event);
    const auto item = itemAt(help_event->pos());
    const auto is_disabled = item && !item->flags().testFlag(Qt::ItemIsEnabled);
    if (is_disabled && !item->toolTip().isEmpty()) {
      // Disabled items do not reliably receive Qt's standard tooltip handling, so display their reason explicitly.
      QToolTip::showText(help_event->globalPos(), item->toolTip(), viewport(), visualItemRect(item));
      return true;
    }
  }

  return super::viewportEvent(_event);
}

QListWidgetItem* SettingsNavigationWidget::getEntry(int _id) const
{
  for (int row = 0; row < count(); ++row) {
    const auto item = this->item(row);
    if (item->data(Qt::UserRole) == _id) {
      return item;
    }
  }

  throw std::runtime_error("Navigation entry not found.");
}

void SettingsNavigationWidget::setListItemEnabled(QListWidgetItem* _item, bool _enabled)
{
  constexpr auto kEnableFlags = Qt::ItemIsSelectable | Qt::ItemIsEnabled;

  if (_enabled) {
    _item->setFlags(_item->flags() | kEnableFlags);
  }
  else {
    _item->setFlags(_item->flags() & ~kEnableFlags);
  }
}

void SettingsNavigationWidget::onCurrentItemChanged(QListWidgetItem* _item)
{
  if (!_item) {
    return;
  }

  const auto id = _item->data(Qt::UserRole);
  if (id.isValid()) {
    Q_EMIT currentEntryChanged(id.toInt());
  }
}
}  // namespace sa
}  // namespace gui
}  // namespace tobas
