// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QWidget>

namespace tobas
{
namespace qt
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

  /* Compute the maximum point size for text that fits within the frame. */
  int calcMaxTextPointSize(const QString& text, const QPoint& center) const;

  /* Set background color. */
  void setBackgroundColor(QPalette::ColorRole cr);

protected:
  /* Draw text. */
  void drawText(QPainter& painter, const QString& text, const QPoint& center, const QFont& font);

  /* Draw text. */
  void drawText(QPainter& painter, const QString& text, const QPoint& center, int point_size);

  /* Draw text at the maximum size that fits within the frame. */
  void drawMaximumText(QPainter& painter, const QString& text, const QPoint& center);

  /* Draw text at the maximum size that fits within the frame. */
  void drawMaximumText(QPainter& painter, const QString& text);
};
}  // namespace qt
}  // namespace tobas
