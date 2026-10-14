// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_qt_tools/widgets/circle_widget.hpp"

#include <QDebug>
#include <QPainter>

namespace tobas::qt
{
CircleWidget::CircleWidget(QWidget* _parent) : super(_parent)
{
}

CircleWidget::CircleWidget(const QString& _text, QWidget* _parent) : super(_parent), text_(_text)
{
}

void CircleWidget::setFillColor(Qt::GlobalColor _color)
{
  fill_color_ = _color;
  update();
}

void CircleWidget::setLineColor(Qt::GlobalColor _color)
{
  line_color_ = _color;
  update();
}

void CircleWidget::setLineWidth(int _width)
{
  if (_width < 0) {
    qWarning() << "Line width must be non-negative.";
    return;
  }

  line_width_ = _width;
  update();
}

void CircleWidget::setText(const QString& _text)
{
  text_ = _text;
  update();
}

void CircleWidget::setTextPointSize(int _psize)
{
  if (_psize < 0) {
    qWarning() << "Text point size must be non-negative.";
    return;
  }

  text_psize_ = _psize;
  update();
}

int CircleWidget::getDiameter() const
{
  return std::min(width(), height());
}

int CircleWidget::getRadius() const
{
  return getDiameter() / 2;
}

int CircleWidget::calcMaxTextPointSize() const
{
  const auto diameter = getDiameter();

  int psize = 1;
  auto font = this->font();

  while (true) {
    font.setPointSize(psize++);
    QFontMetrics fm(font);

    const auto text_width = fm.horizontalAdvance(text_);
    if (text_width > 0.9 * diameter) {
      break;
    }
  }

  return psize;
}

void CircleWidget::paintEvent(QPaintEvent*)
{
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);

  drawCircle(painter);

  if (!text_.isEmpty()) {
    drawText(painter);
  }
}

void CircleWidget::drawCircle(QPainter& _painter)
{
  _painter.save();

  const auto center = getCenter();
  const auto radius = getRadius() - line_width_ / 2;

  // Set the fill brush.
  if (fill_color_ != Qt::transparent) {
    QRadialGradient gradient(center, radius);
    gradient.setColorAt(0, QColor(fill_color_).lighter());  // Center.
    gradient.setColorAt(1, fill_color_);                    // Outer side.
    _painter.setBrush(QBrush(gradient));
  }
  else {
    _painter.setBrush(Qt::NoBrush);
  }

  // Set the outline pen.
  if (line_color_ != Qt::transparent && line_width_ > 0) {
    _painter.setPen(QPen(line_color_, line_width_, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));
  }
  else {
    _painter.setPen(Qt::NoPen);
  }

  // Draw a circle by specifying center and radius.
  _painter.drawEllipse(center, radius, radius);

  _painter.restore();
}

void CircleWidget::drawText(QPainter& _painter)
{
  _painter.save();

  const auto center = getCenter();

  if (text_psize_ > 0) {
    super::drawText(_painter, text_, center, text_psize_);
  }
  else {
    drawMaximumText(_painter, text_, center);
  }

  _painter.restore();
}
}  // namespace tobas::qt
