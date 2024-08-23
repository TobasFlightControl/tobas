#include "../include/tobas_qt_tools/util.hpp"

namespace qt
{
void blockSignalsRec(QObject* obj, bool block)
{
  obj->blockSignals(block);
  for (auto child : obj->children())
    blockSignalsRec(child, block);
}

void placeCenter(QWidget* widget, QVBoxLayout* rows)
{
  auto cols = new QHBoxLayout();
  cols->addWidget(widget);
  cols->setAlignment(widget, Qt::AlignCenter);
  rows->addLayout(cols);
}

QVBoxLayout* createFixedWidthQVBoxLayout(int width, QBoxLayout* parent)
{
  auto widget = new QWidget();
  widget->setFixedWidth(width);
  parent->addWidget(widget);

  auto res = new QVBoxLayout();
  widget->setLayout(res);

  return res;
}

QHBoxLayout* createFixedHeightQHBoxLayout(int height, QBoxLayout* parent)
{
  auto widget = new QWidget();
  widget->setFixedHeight(height);
  parent->addWidget(widget);

  auto res = new QHBoxLayout();
  widget->setLayout(res);

  return res;
}
}  // namespace qt
