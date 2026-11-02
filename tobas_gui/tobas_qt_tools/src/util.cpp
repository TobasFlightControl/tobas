// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_qt_tools/util.hpp"

#include "tobas_qt_tools/widgets/scroll_area.hpp"

namespace tobas::qt
{
void blockSignalsRec(QObject* _obj, bool _block)
{
  _obj->blockSignals(_block);
  for (auto child : _obj->children()) {
    blockSignalsRec(child, _block);
  }
}

void addWidgetCenter(QWidget* _widget, QVBoxLayout* _rows, int _stretch)
{
  const auto cols = new QHBoxLayout();
  cols->addWidget(_widget);
  cols->setAlignment(_widget, Qt::AlignHCenter);
  _rows->addLayout(cols, _stretch);
}

void addWidgetCenter(QWidget* _widget, QHBoxLayout* _cols, int _stretch)
{
  const auto rows = new QVBoxLayout();
  rows->addWidget(_widget);
  rows->setAlignment(_widget, Qt::AlignVCenter);
  _cols->addLayout(rows, _stretch);
}

void addSpacing(QVBoxLayout* _rows, int _height, QSizePolicy::Policy _v_policy)
{
  const auto spacer = new QSpacerItem(0, _height, QSizePolicy::Minimum, _v_policy);
  _rows->addSpacerItem(spacer);
}

void addSpacing(QHBoxLayout* _cols, int _width, QSizePolicy::Policy _h_policy)
{
  const auto spacer = new QSpacerItem(_width, 0, _h_policy, QSizePolicy::Minimum);
  _cols->addSpacerItem(spacer);
}

QGroupBox* makeGroup(const QString& _title, QWidget* _widget, int _margin)
{
  const auto group = new QGroupBox(_title);
  const auto rows = new QVBoxLayout(group);
  rows->setContentsMargins(_margin, _margin, _margin, _margin);
  rows->setSpacing(0);
  rows->addWidget(_widget);
  return group;
}

QVBoxLayout* createFixedWidthQVBoxLayout(int _width, QBoxLayout* _parent)
{
  // parent > widget > layout

  const auto widget = new QWidget();
  widget->setFixedWidth(_width);
  _parent->addWidget(widget);

  const auto layout = new QVBoxLayout(widget);

  return layout;
}

QHBoxLayout* createFixedHeightQHBoxLayout(int _height, QBoxLayout* _parent)
{
  // parent > widget > layout

  const auto widget = new QWidget();
  widget->setFixedHeight(_height);
  _parent->addWidget(widget);

  const auto layout = new QHBoxLayout(widget);

  return layout;
}

void clearLayout(QLayout* _layout)
{
  while (QLayoutItem* item = _layout->takeAt(0)) {
    if (QWidget* widget = item->widget()) {
      widget->hide();
      _layout->removeWidget(widget);
      delete widget;
    }
    else if (QLayout* sub_layout = item->layout()) {
      clearLayout(sub_layout);  // Recursively delete sub-layouts.
      delete sub_layout;
    }
    delete item;
  }
}

QVBoxLayout* createScrollableQVBoxLayout(QBoxLayout* _parent)
{
  const auto scroll_area = new ScrollArea();
  _parent->addWidget(scroll_area);

  const auto content = new QWidget();
  const auto rows = new QVBoxLayout(content);
  scroll_area->setWidget(content);

  return rows;
}
}  // namespace tobas::qt
