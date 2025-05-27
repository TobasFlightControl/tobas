#include "tobas_qt_tools/widgets/tab_widget.hpp"

#include <QWheelEvent>

#include <tobas_std_tools/check.hpp>

#include "tobas_qt_tools/cast.hpp"

namespace qt
{
void TabBar::enableWheelEvent(bool enable)
{
  enable_wheel_event_ = enable;
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

TabWidget::TabWidget(QWidget* parent) : super(parent)
{
  setTabBar(new TabBar());
}

void TabWidget::enableWheelEvent(bool enable)
{
  const auto tab_bar = qPointerCast<TabBar>(tabBar());
  tab_bar->enableWheelEvent(enable);
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
}  // namespace qt
