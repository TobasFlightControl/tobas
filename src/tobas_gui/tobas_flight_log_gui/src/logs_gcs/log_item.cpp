// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_flight_log_gui/logs_gcs/log_item.hpp"

#include <QColor>
#include <QCursor>
#include <QHBoxLayout>
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

  close_menu_timer_ = new QTimer(this);
  close_menu_timer_->setSingleShot(true);
  close_menu_timer_->setInterval(200);

  setMouseTracking(true);
  installEventFilter(this);

  // Layout
  const auto cols = new QHBoxLayout(this);
  cols->addWidget(log_name_, 1);
  cols->addWidget(indicator, 0);

  // Connection
  connect(actions_menu_, &QMenu::aboutToShow, this, [this]() { update(); });
  connect(actions_menu_, &QMenu::aboutToHide, this, [this]() { update(); });
  connect(close_menu_timer_, &QTimer::timeout, this, &self::onCloseTimerTimeout);
}

QString FlightLogItemWidgetGCS::logName() const
{
  return log_name_->text();
}

bool FlightLogItemWidgetGCS::eventFilter(QObject* _watched, QEvent* _event)
{
  if (_watched == this && _event->type() == QEvent::Enter) {
    showActionsMenu();
  }

  if (_watched == this || _watched == actions_menu_) {
    if (_event->type() == QEvent::Enter || _event->type() == QEvent::Leave) {
      update();
    }

    if (_event->type() == QEvent::Enter || _event->type() == QEvent::MouseMove) {
      if (isCursorOverActions()) {
        close_menu_timer_->stop();
      }
      else if (!close_menu_timer_->isActive()) {
        close_menu_timer_->start();
      }
    }
    else if (_event->type() == QEvent::Leave) {
      // Allow the pointer to move from the list item into the menu before closing it.
      close_menu_timer_->start();
    }
  }

  return super::eventFilter(_watched, _event);
}

void FlightLogItemWidgetGCS::paintEvent(QPaintEvent* _event)
{
  super::paintEvent(_event);

  QPainter painter(this);
  // Preserve the selection background drawn by the list.

  // Keep the active row highlighted while its action menu is open.
  if (underMouse() || actions_menu_->isVisible()) {
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
  close_menu_timer_->stop();
  if (!actions_menu_->isVisible()) {
    actions_menu_->popup(mapToGlobal(QPoint(width(), 0)));
  }
}

void FlightLogItemWidgetGCS::onCloseTimerTimeout()
{
  if (!isCursorOverActions()) {
    actions_menu_->hide();
  }
}
}  // namespace tobas::gui::log
