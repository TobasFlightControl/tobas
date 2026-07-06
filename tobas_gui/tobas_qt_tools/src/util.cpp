// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_qt_tools/util.hpp"

#include "tobas_qt_tools/widgets/scroll_area.hpp"

namespace tobas
{
namespace qt
{
void blockSignalsRec(QObject* obj, bool block)
{
  obj->blockSignals(block);
  for (auto child : obj->children()) {
    blockSignalsRec(child, block);
  }
}

void addWidgetCenter(QWidget* widget, QVBoxLayout* rows, int stretch)
{
  const auto cols = new QHBoxLayout();
  cols->addWidget(widget);
  cols->setAlignment(widget, Qt::AlignHCenter);
  rows->addLayout(cols, stretch);
}

void addWidgetCenter(QWidget* widget, QHBoxLayout* cols, int stretch)
{
  const auto rows = new QVBoxLayout();
  rows->addWidget(widget);
  rows->setAlignment(widget, Qt::AlignVCenter);
  cols->addLayout(rows, stretch);
}

void addSpacing(QVBoxLayout* rows, int height, QSizePolicy::Policy v_policy)
{
  const auto spacer = new QSpacerItem(0, height, QSizePolicy::Minimum, v_policy);
  rows->addSpacerItem(spacer);
}

void addSpacing(QHBoxLayout* cols, int width, QSizePolicy::Policy h_policy)
{
  const auto spacer = new QSpacerItem(width, 0, h_policy, QSizePolicy::Minimum);
  cols->addSpacerItem(spacer);
}

QGroupBox* makeGroup(const QString& title, QWidget* widget, int margin)
{
  const auto group = new QGroupBox(title);
  const auto rows = new QVBoxLayout();
  rows->setContentsMargins(margin, margin, margin, margin);
  rows->setSpacing(0);
  rows->addWidget(widget);
  group->setLayout(rows);
  return group;
}

QVBoxLayout* createFixedWidthQVBoxLayout(int width, QBoxLayout* parent)
{
  // parent > widget > layout

  const auto widget = new QWidget();
  widget->setFixedWidth(width);
  parent->addWidget(widget);

  const auto layout = new QVBoxLayout();
  widget->setLayout(layout);

  return layout;
}

QHBoxLayout* createFixedHeightQHBoxLayout(int height, QBoxLayout* parent)
{
  // parent > widget > layout

  const auto widget = new QWidget();
  widget->setFixedHeight(height);
  parent->addWidget(widget);

  const auto layout = new QHBoxLayout();
  widget->setLayout(layout);

  return layout;
}

void clearLayout(QLayout* layout)
{
  while (QLayoutItem* item = layout->takeAt(0)) {
    if (QWidget* widget = item->widget()) {
      widget->hide();
      layout->removeWidget(widget);
      delete widget;
    }
    else if (QLayout* sub_layout = item->layout()) {
      clearLayout(sub_layout);  // Recursively delete sub-layouts.
      delete sub_layout;
    }
    delete item;
  }
}

QVBoxLayout* createScrollableQVBoxLayout(QBoxLayout* parent)
{
  const auto scroll_area = new ScrollArea();
  parent->addWidget(scroll_area);

  const auto rows = new QVBoxLayout();
  scroll_area->setLayout(rows);

  return rows;
}
}  // namespace qt
}  // namespace tobas
