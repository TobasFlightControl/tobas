// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QWidget>

namespace tobas::qt
{
/**
 * ===== Differences from `QWidget` =====
 * - Additional methods
 */
class Widget : public QWidget
{
  Q_OBJECT

  using super = QWidget;

public:
  using super::QWidget;

  QPoint getCenter() const;

  /** Compute the maximum point size for text that fits within the frame. */
  int calcMaxTextPointSize(const QString& _text, const QPoint& _center) const;

  /** Set background color. */
  void setBackgroundColor(QPalette::ColorRole _cr);

protected:
  /** Draw text. */
  void drawText(QPainter& _painter, const QString& _text, const QPoint& _center, const QFont& _font);

  /** Draw text. */
  void drawText(QPainter& _painter, const QString& _text, const QPoint& _center, int _point_size);

  /** Draw text at the maximum size that fits within the frame. */
  void drawMaximumText(QPainter& _painter, const QString& _text, const QPoint& _center);

  /** Draw text at the maximum size that fits within the frame. */
  void drawMaximumText(QPainter& _painter, const QString& _text);
};
}  // namespace tobas::qt
