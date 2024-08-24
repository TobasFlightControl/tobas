#include <QCoreApplication>

#include "tobas_qt_tools/widgets/stacked_widget.hpp"

namespace qt
{
void StackedWidget::clear()
{
  while (count() > 0)
  {
    auto tar_widget = widget(0);
    removeWidget(tar_widget);
    tar_widget->deleteLater();
  }
}

void StackedWidget::setCurrentIndex(int index)
{
  // インデックスを更新
  super::setCurrentIndex(index);

  // Qtのイベントループを更新
  QCoreApplication::processEvents();

  // 画面を更新
  update();
}
}  // namespace qt
