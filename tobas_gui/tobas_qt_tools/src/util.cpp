#include <tobas_std_tools/console.hpp>

#include "tobas_qt_tools/util.hpp"

namespace qt
{
void blockSignalsRec(QObject* obj, bool block)
{
  obj->blockSignals(block);
  for (auto child : obj->children())
    blockSignalsRec(child, block);
}

void addWidgetCenter(QWidget* widget, QVBoxLayout* rows)
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

void clearLayout(QLayout* layout)
{
  while (QLayoutItem* item = layout->takeAt(0))
  {
    if (QWidget* widget = item->widget())
    {
      widget->hide();
      layout->removeWidget(widget);
      delete widget;
    }
    else if (QLayout* sub_layout = item->layout())
    {
      clearLayout(sub_layout);  // 再帰的にサブレイアウトを削除
      delete sub_layout;
    }
    else if (item->spacerItem())
    {
      delete item->spacerItem();  // スペーサーアイテムを削除
    }
    else
    {
      PRINT_WARN("Unknown layout item type. Failed to delete it.");
    }
    delete item;
  }
}
}  // namespace qt
