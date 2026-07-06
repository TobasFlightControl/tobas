// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_qt_tools/widgets/widget.hpp>

namespace tobas
{
namespace gui
{
namespace sc
{
class FaceCircleWidget : public qt::Widget
{
  Q_OBJECT

  using self = FaceCircleWidget;
  using super = qt::Widget;

  static constexpr int kLineWidth = 10;
  static constexpr auto kFillColorComplete = Qt::green;
  static constexpr auto kFillColorIncomplete = Qt::gray;
  static constexpr auto kFillColorProgress = Qt::yellow;
  static constexpr auto kLineColorSelected = Qt::red;
  static constexpr auto kLineColorDeselected = Qt::black;

public:
  explicit FaceCircleWidget(const QString& text);

  void setTextPointSize(int psize);

  void setProgress(double progress);
  void setSelected(bool selected);

  int calcMaxTextPointSize() const;

protected:
  void paintEvent(QPaintEvent* event) override;

private:
  const QString text_ = "";
  int text_psize_ = 0;

  double progress_ = 0.0;
  bool completed_ = false;
  bool selected_ = false;

  int getDiameter() const;
  int getRadius() const;

  void drawCircle(QPainter& painter, const QColor& color);
  void drawCircleLowerHalf(QPainter& painter, const QColor& color, double rate);
  void drawOutline(QPainter& painter);
  void drawText(QPainter& painter);
};
}  // namespace sc
}  // namespace gui
}  // namespace tobas
