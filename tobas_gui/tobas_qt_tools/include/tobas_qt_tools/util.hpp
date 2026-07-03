// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QHBoxLayout>
#include <QObject>
#include <QVBoxLayout>
#include <QWidget>

namespace tobas
{
namespace qt
{
/* Recursively scan child widgets and block all signals. */
void blockSignalsRec(QObject* obj, bool block);

/* Add the widget to the center of the layout. */
void addWidgetCenter(QWidget* widget, QVBoxLayout* rows, int stretch = 0);

/* Add the widget to the center of the layout. */
void addWidgetCenter(QWidget* widget, QHBoxLayout* cols, int stretch = 0);

/* Insert a spacer with a size policy. */
void addSpacing(QVBoxLayout* rows, int height, QSizePolicy::Policy v_policy);

/* Insert a spacer with a size policy. */
void addSpacing(QHBoxLayout* cols, int width, QSizePolicy::Policy h_policy);

/* Create a fixed-width `QVBoxLayout`. */
QVBoxLayout* createFixedWidthQVBoxLayout(int width, QBoxLayout* parent);

/* Create a fixed-height `QVBoxLayout`. */
QHBoxLayout* createFixedHeightQHBoxLayout(int height, QBoxLayout* parent);

/* Delete all items in the layout. */
void clearLayout(QLayout* layout);

/* Create a `QVBoxLayout` with a `ScrollWidget` between them. */
QVBoxLayout* createScrollableQVBoxLayout(QBoxLayout* parent);
}  // namespace qt
}  // namespace tobas
