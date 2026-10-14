// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include "./widget.hpp"

namespace tobas::qt
{
class CircleWidget : public qt::Widget
{
  Q_OBJECT

  using super = qt::Widget;

public:
  explicit CircleWidget(QWidget* _parent = nullptr);
  explicit CircleWidget(const QString& _text, QWidget* _parent = nullptr);

  void setFillColor(Qt::GlobalColor _color);
  void setLineColor(Qt::GlobalColor _color);
  void setLineWidth(int _width);
  void setText(const QString& _text);
  void setTextPointSize(int _psize);

  int getDiameter() const;
  int getRadius() const;

  int calcMaxTextPointSize() const;

protected:
  void paintEvent(QPaintEvent* _event) override;

private:
  Qt::GlobalColor fill_color_ = Qt::transparent;
  Qt::GlobalColor line_color_ = Qt::transparent;
  int line_width_ = 0;
  QString text_ = "";
  int text_psize_ = 0;

  void drawCircle(QPainter& _painter);
  void drawText(QPainter& _painter);
};
}  // namespace tobas::qt
