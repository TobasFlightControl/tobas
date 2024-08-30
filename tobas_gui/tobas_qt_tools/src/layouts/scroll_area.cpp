#include "tobas_qt_tools/layouts/scroll_area.hpp"

namespace qt
{
ScrollableVBoxLayout::ScrollableVBoxLayout(QWidget* parent) : QVBoxLayout(parent)
{
  auto scroll_area = new ScrollArea();
  addWidget(scroll_area);
  rows_ = new QVBoxLayout(scroll_area);
}

void ScrollableVBoxLayout::addWidget(QWidget* widget)
{
  rows_->addWidget(widget);
}

void ScrollableVBoxLayout::addLayout(QLayout* layout)
{
  rows_->addLayout(layout);
}

void ScrollableVBoxLayout::addStretch()
{
  rows_->addStretch();
}
}  // namespace qt
