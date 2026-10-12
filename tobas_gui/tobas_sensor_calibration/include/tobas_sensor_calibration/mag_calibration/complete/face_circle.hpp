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

  using super = qt::Widget;

public:
  explicit FaceCircleWidget(const QString& _text);

  void setTextPointSize(int _psize);

  void setProgress(double _progress);
  void setSelected(bool _selected);

  int calcMaxTextPointSize() const;

protected:
  void paintEvent(QPaintEvent* _event) override;

private:
  const QString text_ = "";
  int text_psize_ = 0;

  double progress_ = 0.0;
  bool completed_ = false;
  bool selected_ = false;

  int getDiameter() const;
  int getRadius() const;

  void drawCircle(QPainter& _painter, const QColor& _color);
  void drawCircleLowerHalf(QPainter& _painter, const QColor& _color, double _rate);
  void drawOutline(QPainter& _painter);
  void drawText(QPainter& _painter);
};
}  // namespace sc
}  // namespace gui
}  // namespace tobas
