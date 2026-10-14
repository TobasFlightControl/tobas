// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QGroupBox>
#include <QHBoxLayout>
#include <QObject>
#include <QVBoxLayout>
#include <QWidget>

namespace tobas
{
namespace qt
{
/** Recursively scan child widgets and block all signals. */
void blockSignalsRec(QObject* _obj, bool _block);

/** Add the widget to the center of the layout. */
void addWidgetCenter(QWidget* _widget, QVBoxLayout* _rows, int _stretch = 0);

/** Add the widget to the center of the layout. */
void addWidgetCenter(QWidget* _widget, QHBoxLayout* _cols, int _stretch = 0);

/** Insert a spacer with a size policy. */
void addSpacing(QVBoxLayout* _rows, int _height, QSizePolicy::Policy _v_policy);

/** Insert a spacer with a size policy. */
void addSpacing(QHBoxLayout* _cols, int _width, QSizePolicy::Policy _h_policy);

/** Create a titled group box containing the widget. */
QGroupBox* makeGroup(const QString& _title, QWidget* _widget, int _margin = 0);

/** Create a fixed-width `QVBoxLayout`. */
QVBoxLayout* createFixedWidthQVBoxLayout(int _width, QBoxLayout* _parent);

/** Create a fixed-height `QVBoxLayout`. */
QHBoxLayout* createFixedHeightQHBoxLayout(int _height, QBoxLayout* _parent);

/** Delete all items in the layout. */
void clearLayout(QLayout* _layout);

/** Create a `QVBoxLayout` with a `ScrollWidget` between them. */
QVBoxLayout* createScrollableQVBoxLayout(QBoxLayout* _parent);
}  // namespace qt
}  // namespace tobas
