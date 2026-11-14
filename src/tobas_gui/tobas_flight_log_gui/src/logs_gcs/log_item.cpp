// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_flight_log_gui/logs_gcs/log_item.hpp"

#include <QColor>
#include <QCursor>
#include <QHBoxLayout>
#include <QObject>
#include <QPainter>
#include <QPalette>
#include <QPoint>

namespace tobas::gui::log
{
FlightLogItemWidgetGCS::FlightLogItemWidgetGCS(const QString& _log_name)
{
  log_name_ = new QLabel(_log_name);
  log_name_->setAttribute(Qt::WA_TransparentForMouseEvents);

  const auto indicator = new QLabel(">");
  indicator->setAttribute(Qt::WA_TransparentForMouseEvents);

  actions_menu_ = new QMenu(this);
  actions_menu_->addAction("Show", this, [this]() { Q_EMIT showButtonClicked(logName()); });
  actions_menu_->addAction("Export", this, [this]() { Q_EMIT exportButtonClicked(logName()); });
  actions_menu_->addAction("Delete", this, [this]() { Q_EMIT deleteButtonClicked(logName()); });
  actions_menu_->installEventFilter(this);

  setMouseTracking(true);
  installEventFilter(this);

  // Layout
  const auto cols = new QHBoxLayout(this);
  cols->addWidget(log_name_, 1);
  cols->addWidget(indicator, 0);
}

QString FlightLogItemWidgetGCS::logName() const
{
  return log_name_->text();
}

bool FlightLogItemWidgetGCS::eventFilter(QObject* _watched, QEvent* _event)
{
  const auto type = _event->type();

  // Open the action menu when the cursor enters this row.
  if (_watched == this && type == QEvent::Enter) {
    showActionsMenu();
  }

  if (_watched == this || _watched == actions_menu_) {
    // Repaint on hover and visibility changes to clear stale highlighting.
    if (type == QEvent::Enter || type == QEvent::Leave || type == QEvent::Show || type == QEvent::Hide) {
      update();
    }

    // Allow movement between the row and its menu; close the menu only when the cursor leaves both.
    if ((type == QEvent::Enter || type == QEvent::Leave || type == QEvent::MouseMove) && !isCursorOverActions()) {
      actions_menu_->hide();
      if (_watched == actions_menu_) {
        return true;  // Do not pass the event on to the menu that was just closed.
      }
    }
  }

  return super::eventFilter(_watched, _event);
}

void FlightLogItemWidgetGCS::paintEvent(QPaintEvent* _event)
{
  super::paintEvent(_event);

  QPainter painter(this);

  // Highlight the row while the cursor is over it or its action menu.
  if (isCursorOverActions()) {
    auto highlight = palette().color(QPalette::Highlight);
    highlight.setAlpha(64);
    painter.fillRect(rect(), highlight);
  }
}

bool FlightLogItemWidgetGCS::isCursorOverActions() const
{
  const auto pos = QCursor::pos();
  const auto in_item = rect().contains(mapFromGlobal(pos));
  const auto in_menu = actions_menu_->isVisible() && actions_menu_->rect().contains(actions_menu_->mapFromGlobal(pos));
  return in_item || in_menu;
}

void FlightLogItemWidgetGCS::showActionsMenu()
{
  if (!actions_menu_->isVisible()) {
    actions_menu_->popup(mapToGlobal(QPoint(width(), 0)));
  }
}
}  // namespace tobas::gui::log
