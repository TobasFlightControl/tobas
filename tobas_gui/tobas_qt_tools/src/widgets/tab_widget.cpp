// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_qt_tools/widgets/tab_widget.hpp"

#include <QStyleOptionTab>
#include <QStylePainter>

#include <tobas_std_tools/check.hpp>

#include "tobas_qt_tools/cast.hpp"

namespace tobas
{
namespace qt
{
TabBar::TabBar()
{
  // Dim text when disabled.
  setStyleSheet("QTabBar::tab:disabled { color: palette(midlight); }");
}

void TabBar::enableWheelEvent(bool enable)
{
  enable_wheel_event_ = enable;
}

void TabBar::setTabBackgroundColor(int index, const QColor& color)
{
  colors_[index] = color;
  update();
}

void TabBar::clearTabBackgroundColor(int index)
{
  if (!colors_.contains(index)) {
    qWarning().nospace() << "No color is set for tab " << index << ".";
    return;
  }

  colors_.remove(index);
  update();
}

void TabBar::wheelEvent(QWheelEvent* event)
{
  if (enable_wheel_event_) {
    super::wheelEvent(event);
  }
  else {
    event->ignore();
  }
}

void TabBar::paintEvent(QPaintEvent*)
{
  QStylePainter painter(this);
  QStyleOptionTab opt;

  for (int i = 0; i < count(); ++i) {
    initStyleOption(&opt, i);

    if (colors_.contains(i)) {
      opt.palette.setColor(QPalette::Button, colors_.value(i));
    }

    painter.drawControl(QStyle::CE_TabBarTabShape, opt);
    painter.drawControl(QStyle::CE_TabBarTabLabel, opt);
  }
}

TabWidget::TabWidget(QWidget* parent) : super(parent)
{
  tab_bar_ = new TabBar();
  setTabBar(tab_bar_);
}

void TabWidget::enableWheelEvent(bool enable)
{
  const auto tab_bar = qPointerCast<TabBar>(tabBar());
  tab_bar->enableWheelEvent(enable);
}

void TabWidget::setTabBackgroundColor(int index, const QColor& color)
{
  tab_bar_->setTabBackgroundColor(index, color);
}

void TabWidget::clearTabBackgroundColor(int index)
{
  tab_bar_->clearTabBackgroundColor(index);
}

void TabWidget::setTabEnabled(QWidget* tab, bool enabled)
{
  const auto idx = indexOf(tab);
  TOBAS_CHECK(idx >= 0);
  tabBar()->setTabEnabled(idx, enabled);
}

void TabWidget::setTabVisible(QWidget* tab, bool visible)
{
  const auto idx = indexOf(tab);
  TOBAS_CHECK(idx >= 0);
  tabBar()->setTabVisible(idx, visible);
}

void TabWidget::setTabWidth(int width)
{
  const auto qss = std::format("QTabBar::tab {{ width: {}px; }}", width);
  setStyleSheet(QString::fromStdString(qss));
}

void TabWidget::setTabHeight(int height)
{
  const auto qss = std::format("QTabBar::tab {{ height: {}px; }}", height);
  setStyleSheet(QString::fromStdString(qss));
}

void TabWidget::setTabSize(int width, int height)
{
  const auto qss = std::format("QTabBar::tab {{ width: {}px; height: {}px; }}", width, height);
  setStyleSheet(QString::fromStdString(qss));
}

void TabWidget::removeAllTabs()
{
  while (count() > 0) {
    const auto first_widget = widget(0);  // Get the first page.
    removeTab(0);                         // Remove from the tab bar.
    first_widget->deleteLater();          // Free memory.
  }
}
}  // namespace qt
}  // namespace tobas
