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

void TabBar::enableWheelEvent(bool _enable)
{
  enable_wheel_event_ = _enable;
}

void TabBar::setTabBackgroundColor(int _index, const QColor& _color)
{
  colors_[_index] = _color;
  update();
}

void TabBar::clearTabBackgroundColor(int _index)
{
  if (!colors_.contains(_index)) {
    qWarning().nospace() << "No color is set for tab " << _index << ".";
    return;
  }

  colors_.remove(_index);
  update();
}

void TabBar::wheelEvent(QWheelEvent* _event)
{
  if (enable_wheel_event_) {
    super::wheelEvent(_event);
  }
  else {
    _event->ignore();
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

TabWidget::TabWidget(QWidget* _parent) : super(_parent)
{
  tab_bar_ = new TabBar();
  setTabBar(tab_bar_);
}

void TabWidget::enableWheelEvent(bool _enable)
{
  const auto tab_bar = qPointerCast<TabBar>(tabBar());
  tab_bar->enableWheelEvent(_enable);
}

void TabWidget::setTabBackgroundColor(int _index, const QColor& _color)
{
  tab_bar_->setTabBackgroundColor(_index, _color);
}

void TabWidget::clearTabBackgroundColor(int _index)
{
  tab_bar_->clearTabBackgroundColor(_index);
}

void TabWidget::setTabEnabled(QWidget* _tab, bool _enabled)
{
  _tab->setEnabled(_enabled);

  const auto idx = indexOf(_tab);
  TOBAS_CHECK(idx >= 0);
  tabBar()->setTabEnabled(idx, _enabled);
}

void TabWidget::setTabVisible(QWidget* _tab, bool _visible)
{
  const auto idx = indexOf(_tab);
  TOBAS_CHECK(idx >= 0);
  tabBar()->setTabVisible(idx, _visible);
}

void TabWidget::setTabWidth(int _width)
{
  const auto qss = std::format("QTabBar::tab {{ width: {}px; }}", _width);
  setStyleSheet(QString::fromStdString(qss));
}

void TabWidget::setTabHeight(int _height)
{
  const auto qss = std::format("QTabBar::tab {{ height: {}px; }}", _height);
  setStyleSheet(QString::fromStdString(qss));
}

void TabWidget::setTabSize(int _width, int _height)
{
  const auto qss = std::format("QTabBar::tab {{ width: {}px; height: {}px; }}", _width, _height);
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
