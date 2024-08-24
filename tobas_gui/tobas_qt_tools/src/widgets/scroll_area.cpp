#include "tobas_qt_tools/widgets/scroll_area.hpp"

namespace qt
{
ScrollArea::ScrollArea(QWidget* parent) : QScrollArea(parent)
{
  setWidgetResizable(true);
}

void ScrollArea::setLayout(QLayout* layout)
{
  // スクロールエリアに入れられるウィジェットは1つのみだから，Layoutを使うためには空のウィジェットを挟む必要がある
  auto inner_widget = new QWidget();
  setWidget(inner_widget);
  inner_widget->setLayout(layout);
}
}  // namespace qt
