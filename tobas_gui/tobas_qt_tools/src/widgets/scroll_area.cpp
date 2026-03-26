#include "tobas_qt_tools/widgets/scroll_area.hpp"

namespace tobas
{
namespace qt
{
ScrollArea::ScrollArea(QWidget* parent) : QScrollArea(parent)
{
  setWidgetResizable(true);
}

void ScrollArea::setLayout(QLayout* layout)
{
  if (widget()) {
    throw std::runtime_error("Widget already set.");
  }

  // スクロールエリアに入れられるウィジェットは1つのみだから，Layoutを使うためには空のウィジェットを挟む必要がある
  const auto inner_widget = new QWidget();
  setWidget(inner_widget);
  inner_widget->setLayout(layout);
}

void ScrollArea::setBackgroundTransparent()
{
  const auto _viewport = viewport();
  if (!_viewport) {
    throw std::runtime_error("Viewport not found.");
  }

  const auto _widget = widget();
  if (!_widget) {
    throw std::runtime_error("Widget not found.");
  }

  setAttribute(Qt::WA_TranslucentBackground);

  _viewport->setAttribute(Qt::WA_TranslucentBackground);
  _viewport->setAutoFillBackground(false);

  _widget->setAttribute(Qt::WA_TranslucentBackground);
  _widget->setAutoFillBackground(false);
}
}  // namespace qt
}  // namespace tobas
