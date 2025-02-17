#include <QWheelEvent>

#include <tobas_std_tools/check.hpp>

#include "tobas_qt_tools/widgets/tab_widget.hpp"

namespace qt
{
void TabBarWithNoWheelEvent::wheelEvent(QWheelEvent* event)
{
  event->ignore();
}

void TabWidget::ignoreWheelEvent()
{
  setTabBar(new TabBarWithNoWheelEvent());
}

void TabWidget::switchTab(QWidget* tab)
{
  const auto idx = indexOf(tab);
  TOBAS_CHECK(idx >= 0);
  setCurrentIndex(idx);
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
