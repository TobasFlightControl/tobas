// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_sensor_calibration/mag_calibration/complete/face_circle.hpp"

#include <QDebug>
#include <QPainter>

namespace tobas
{
namespace gui
{
namespace sc
{
FaceCircleWidget::FaceCircleWidget(const QString& text) : text_(text)
{
}

void FaceCircleWidget::setTextPointSize(int psize)
{
  if (psize < 0) {
    qWarning() << "Text point size must be non-negative.";
    return;
  }

  text_psize_ = psize;
  update();
}

void FaceCircleWidget::setProgress(double progress)
{
  progress_ = std::clamp(progress, 0., 1.);
  completed_ = progress >= 1.;
}

void FaceCircleWidget::setSelected(bool selected)
{
  selected_ = selected;
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
    drawCircle(painter, kFillColorComplete);
  }
  else {
    drawCircle(painter, kFillColorIncomplete);
    drawCircleLowerHalf(painter, kFillColorProgress, progress_);
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

void FaceCircleWidget::drawCircle(QPainter& painter, const QColor& color)
{
  painter.save();

  const auto center = getCenter();
  const auto radius = getRadius() - kLineWidth;

  // No line.
  painter.setPen(Qt::NoPen);

  // Set the brush.
  QRadialGradient gradient(center, radius);
  gradient.setColorAt(0, color.lighter());
  gradient.setColorAt(1, color);
  painter.setBrush(QBrush(gradient));

  // Draw.
  painter.drawEllipse(center, radius, radius);

  painter.restore();
}

void FaceCircleWidget::drawCircleLowerHalf(QPainter& painter, const QColor& color, double rate)
{
  painter.save();

  const auto center = getCenter();
  const auto radius = getRadius() - kLineWidth;

  // No line.
  painter.setPen(Qt::NoPen);

  // Set the brush.
  QRadialGradient gradient(center, radius);
  gradient.setColorAt(0, QColor(color).lighter());
  gradient.setColorAt(1, color);
  painter.setBrush(QBrush(gradient));

  // Clip only the lower part.
  const auto top = center.y() + radius * (1 - 2 * rate);
  QRectF clip(0., top, width(), height() - top);
  painter.setClipRect(clip);

  // Draw.
  painter.drawEllipse(center, radius, radius);

  painter.restore();
}

void FaceCircleWidget::drawOutline(QPainter& painter)
{
  painter.save();

  const auto center = getCenter();
  const auto radius = getRadius() - kLineWidth / 2;

  // Set the pen.
  const auto color = selected_ ? kLineColorSelected : kLineColorDeselected;
  painter.setPen(QPen(color, kLineWidth, Qt::SolidLine, Qt::RoundCap, Qt::RoundJoin));

  // No fill.
  painter.setBrush(Qt::NoBrush);

  // Draw.
  painter.drawEllipse(center, radius, radius);

  painter.restore();
}

void FaceCircleWidget::drawText(QPainter& painter)
{
  painter.save();

  const auto center = getCenter();

  if (text_psize_ > 0) {
    super::drawText(painter, text_, center, text_psize_);
  }
  else {
    drawMaximumText(painter, text_, center);
  }

  painter.restore();
}
}  // namespace sc
}  // namespace gui
}  // namespace tobas
