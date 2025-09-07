#include "tobas_qt_tools/widgets/tab_widget.hpp"

#include <QStyleOptionTab>

#include <tobas_std_tools/check.hpp>

#include "tobas_qt_tools/cast.hpp"

namespace qt
{
TabBar::TabBar()
{
  // 無効時に文字を薄くする
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
    qWarning() << "No color is set for tab " << index << ".";
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
  QStyleOptionTab opt;

  for (int i = 0; i < count(); ++i) {
    initStyleOption(&opt, i);

    if (colors_.contains(i)) {
      opt.palette.setColor(QPalette::Button, colors_.value(i));
    }
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
    const auto first_widget = widget(0);  // 先頭ページを取得
    removeTab(0);                         // タブバーから外す
    first_widget->deleteLater();          // メモリを解放
  }
}
}  // namespace qt
