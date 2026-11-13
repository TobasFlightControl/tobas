// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_sensor_calibration/mag_calibration/complete/face_circle.hpp"

#include <QDebug>
#include <QPainter>

namespace tobas::gui::sc
{
namespace
{
constexpr int kLineWidth = 10;
}  // namespace

FaceCircleWidget::FaceCircleWidget(const QString& _text) : text_(_text)
{
}

void FaceCircleWidget::setTextPointSize(int _psize)
{
  if (_psize < 0) {
    qWarning() << "Text point size must be non-negative.";
    return;
  }

  text_psize_ = _psize;
  update();
}

void FaceCircleWidget::setProgress(double _progress)
{
  progress_ = std::clamp(_progress, 0.0, 1.0);
  completed_ = _progress >= 1.0;
}

void FaceCircleWidget::setSelected(bool _selected)
{
  selected_ = _selected;
}

int FaceCircleWidget::calcMaxTextPointSize() const
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

void FaceCircleWidget::paintEvent(QPaintEvent*)
{
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);

  if (completed_) {
    drawCircle(painter, Qt::green);
  }
  else {
    drawCircle(painter, Qt::gray);
    drawCircleLowerHalf(painter, Qt::yellow, progress_);
  }

  drawOutline(painter);

  if (!text_.isEmpty()) {
    drawText(painter);
  }
}

int FaceCircleWidget::getDiameter() const
{
  return std::min(width(), height());
}

int FaceCircleWidget::getRadius() const
{
  return getDiameter() / 2;
}

void FaceCircleWidget::drawCircle(QPainter& _painter, const QColor& _color)
{
  _painter.save();

  const auto center = getCenter();
  const auto radius = getRadius() - kLineWidth;

  // No line.
  _painter.setPen(Qt::NoPen);

  // Set the brush.
  QRadialGradient gradient(center, radius);
  gradient.setColorAt(0, _color.lighter());
  gradient.setColorAt(1, _color);
  _painter.setBrush(QBrush(gradient));

  // Draw.
  _painter.drawEllipse(center, radius, radius);

  _painter.restore();
}

void FaceCircleWidget::drawCircleLowerHalf(QPainter& _painter, const QColor& _color, double _rate)
{
  _painter.save();

  const auto center = getCenter();
  const auto radius = getRadius() - kLineWidth;

  // No line.
  _painter.setPen(Qt::NoPen);

  // Set the brush.
  QRadialGradient gradient(center, radius);
  gradient.setColorAt(0, QColor(_color).lighter());
  gradient.setColorAt(1, _color);
  _painter.setBrush(QBrush(gradient));

  // Clip only the lower part.
  const auto top = center.y() + radius * (1 - 2 * _rate);
  QRectF clip(0.0, top, width(), height() - top);
  _painter.setClipRect(clip);

  // Draw.
  _painter.drawEllipse(center, radius, radius);

  _painter.restore();
}

void FaceCircleWidget::drawOutline(QPainter& _painter)
{
  constexpr auto kLineColorSelected = Qt::red;
  constexpr auto kLineColorDeselected = Qt::black;

  _painter.save();

  const auto center = getCenter();
  const auto radius = getRadius() - kLineWidth / 2;

  // Set the pen.
  const auto color = selected_ ? kLineColorSelected : kLineColorDeselected;
  _painter.setPen(QPen(color, kLineWidth, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));

  // No fill.
  _painter.setBrush(Qt::NoBrush);

  // Draw.
  _painter.drawEllipse(center, radius, radius);

  _painter.restore();
}

void FaceCircleWidget::drawText(QPainter& _painter)
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
}  // namespace tobas::gui::sc
