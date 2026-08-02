// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_sensor_calibration/accel_calibration/level_indicator.hpp"

#include <algorithm>
#include <cmath>

#include <QPainter>
#include <QRadialGradient>

#include <tobas_std_tools/unit_conversions.hpp>

namespace tobas
{
namespace gui
{
namespace sc
{
LevelIndicatorWidget::LevelIndicatorWidget(QWidget* parent) : super(parent)
{
  constexpr double kLpfCutoff = 1.0;  // [Hz]
  acc_raw_lpf_.setCutoffFrequency(kLpfCutoff);
  acc_calib_lpf_.setCutoffFrequency(kLpfCutoff);
}

QSize LevelIndicatorWidget::sizeHint() const
{
  constexpr int kPreferredSize = 300;
  return { kPreferredSize, kPreferredSize };
}

QSize LevelIndicatorWidget::minimumSizeHint() const
{
  constexpr int kMinimumSize = 220;
  return { kMinimumSize, kMinimumSize };
}

void LevelIndicatorWidget::clear()
{
  has_raw_data_ = false;
  has_calib_data_ = false;
  update();
}

void LevelIndicatorWidget::setRawAccel(const kdl::Vector& acc, double dt)
{
  if (!acc.isFinite() || !std::isfinite(dt)) {
    return;
  }

  if (!has_raw_data_ || dt <= 0.0) {
    acc_raw_lpf_.setValue(acc);
    has_raw_data_ = true;
  }
  else {
    acc_raw_lpf_.update(acc, dt);
  }

  update();
}

void LevelIndicatorWidget::setCalibratedAccel(const kdl::Vector& acc, double dt)
{
  if (!acc.isFinite() || !std::isfinite(dt)) {
    return;
  }

  if (!has_calib_data_ || dt <= 0.0) {
    acc_calib_lpf_.setValue(acc);
    has_calib_data_ = true;
  }
  else {
    acc_calib_lpf_.update(acc, dt);
  }

  update();
}

LevelIndicatorWidget::IndicatorPosition LevelIndicatorWidget::calculateIndicatorPosition(
  const kdl::Vector& acc,
  const QPointF& center,
  double radius,
  double marker_radius) const
{
  static constexpr double kMaxTilt = st::deg2rad(10.0);  // [rad]

  const auto horizontal_acc = std::hypot(acc.x(), acc.y());
  const auto tilt = std::atan2(horizontal_acc, acc.z());
  const auto max_distance = std::max(0.0, radius - marker_radius - 3.0);
  const auto distance = std::clamp(tilt / kMaxTilt, 0.0, 1.0) * max_distance;

  auto marker_center = center;
  if (horizontal_acc > 0.0) {
    // The widget is a top view: +X is front and +Y is left.
    marker_center.rx() -= acc.y() / horizontal_acc * distance;
    marker_center.ry() -= acc.x() / horizontal_acc * distance;
  }

  return { marker_center, tilt };
}

void LevelIndicatorWidget::paintEvent(QPaintEvent*)
{
  constexpr int kOuterMargin = 40;

  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);

  const QPointF center(width() / 2.0, height() / 2.0);
  const auto radius = std::max(0.0, std::min(width(), height()) / 2.0 - kOuterMargin);
  const QRectF level_rect(center.x() - radius, center.y() - radius, 2.0 * radius, 2.0 * radius);

  // Draw the level background and outline.
  painter.setPen(QPen(palette().color(QPalette::Mid), 2.0));
  painter.setBrush(palette().color(QPalette::Base));
  painter.drawEllipse(level_rect);

  // Draw the crosshair and concentric tilt guides.
  painter.setPen(QPen(palette().color(QPalette::Mid), 1.0, Qt::DashLine));
  painter.drawLine(QPointF(center.x() - radius, center.y()), QPointF(center.x() + radius, center.y()));
  painter.drawLine(QPointF(center.x(), center.y() - radius), QPointF(center.x(), center.y() + radius));
  painter.drawEllipse(center, radius / 2.0, radius / 2.0);

  const auto target_radius = std::max(6.0, radius / 10.0);
  painter.setPen(QPen(QColor(0, 160, 0), 2.0));
  painter.drawEllipse(center, target_radius, target_radius);

  // Draw orientation labels for the FLU body frame.
  painter.setPen(palette().color(QPalette::Text));
  painter.drawText(
    QRectF(center.x() - 35.0, level_rect.top() - kOuterMargin, 70.0, kOuterMargin), Qt::AlignCenter, "Front");
  painter.drawText(QRectF(center.x() - 35.0, level_rect.bottom(), 70.0, kOuterMargin), Qt::AlignCenter, "Rear");
  painter.drawText(
    QRectF(level_rect.left() - kOuterMargin, center.y() - 15.0, kOuterMargin, 30.0), Qt::AlignCenter, "Left");
  painter.drawText(QRectF(level_rect.right(), center.y() - 15.0, kOuterMargin, 30.0), Qt::AlignCenter, "Right");

  const auto bubble_radius = std::max(8.0, radius / 12.0);
  if (has_raw_data_) {
    const auto raw_position = calculateIndicatorPosition(acc_raw_lpf_.getValue(), center, radius, bubble_radius);

    QRadialGradient bubble_gradient(raw_position.center, bubble_radius);
    bubble_gradient.setColorAt(0.0, QColor(255, 255, 180));
    bubble_gradient.setColorAt(1.0, QColor(220, 180, 0));
    painter.setPen(QPen(QColor(140, 110, 0), 2.0));
    painter.setBrush(bubble_gradient);
    painter.drawEllipse(raw_position.center, bubble_radius, bubble_radius);

    const auto raw_tilt_deg = st::rad2deg(raw_position.tilt);
    painter.setPen(QColor(140, 110, 0));
    painter.drawText(
      QRectF(center.x() - 70.0, level_rect.bottom() - 50.0, 140.0, 22.0),
      Qt::AlignCenter,
      QString("Raw: %1 deg").arg(raw_tilt_deg, 0, 'f', 1));
  }
  else {
    painter.setPen(QColor(140, 110, 0));
    painter.drawText(
      QRectF(center.x() - 70.0, level_rect.bottom() - 50.0, 140.0, 22.0), Qt::AlignCenter, "Raw: Waiting...");
  }

  if (has_calib_data_) {
    const auto result_radius = bubble_radius * 0.8;
    const auto calibrated_position =
      calculateIndicatorPosition(acc_calib_lpf_.getValue(), center, radius, result_radius);

    const QColor result_color(0, 160, 0);
    painter.setPen(QPen(result_color, 3.0));
    painter.setBrush(QColor(0, 200, 0, 80));
    painter.drawEllipse(calibrated_position.center, result_radius, result_radius);
    painter.drawLine(
      QPointF(calibrated_position.center.x() - result_radius, calibrated_position.center.y()),
      QPointF(calibrated_position.center.x() + result_radius, calibrated_position.center.y()));
    painter.drawLine(
      QPointF(calibrated_position.center.x(), calibrated_position.center.y() - result_radius),
      QPointF(calibrated_position.center.x(), calibrated_position.center.y() + result_radius));

    const auto calibrated_tilt_deg = st::rad2deg(calibrated_position.tilt);
    painter.setPen(result_color);
    painter.drawText(
      QRectF(center.x() - 70.0, level_rect.bottom() - 28.0, 140.0, 22.0),
      Qt::AlignCenter,
      QString("Calibrated: %1 deg").arg(calibrated_tilt_deg, 0, 'f', 1));
  }
  else {
    painter.setPen(QColor(0, 160, 0));
    painter.drawText(
      QRectF(center.x() - 70.0, level_rect.bottom() - 28.0, 140.0, 22.0), Qt::AlignCenter, "Calibrated: Waiting...");
  }
}
}  // namespace sc
}  // namespace gui
}  // namespace tobas
