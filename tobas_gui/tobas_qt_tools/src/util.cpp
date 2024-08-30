#include <QVBoxLayout>
#include <QHBoxLayout>

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
  return new QVBoxLayout(widget);
}

QHBoxLayout* createFixedHeightQHBoxLayout(int height, QBoxLayout* parent)
{
  auto widget = new QWidget();
  widget->setFixedHeight(height);
  parent->addWidget(widget);
  return new QHBoxLayout(widget);
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

QWidget* createVerticalWidgetsContainer(const std::vector<QWidget*>& widgets)
{
  // コンテナウィジェットを作成
  const auto container = new QWidget();

  // 垂直レイアウトを作成
  const auto rows = new QVBoxLayout(container);

  // 受け取ったウィジェットをレイアウトに追加
  for (const auto widget : widgets)
    rows->addWidget(widget);

  // コンテナウィジェットを返す
  return container;
}

QWidget* createHorizontalWidgetsContainer(const std::vector<QWidget*>& widgets)
{
  // コンテナウィジェットを作成
  const auto container = new QWidget();

  // 水平レイアウトを作成
  const auto cols = new QHBoxLayout(container);

  // 受け取ったウィジェットをレイアウトに追加
  for (const auto widget : widgets)
    cols->addWidget(widget);

  // コンテナウィジェットを返す
  return container;
}
}  // namespace qt
