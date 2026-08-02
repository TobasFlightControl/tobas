// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_sensor_calibration/accel_calibration/level_indicator.hpp"

#include <algorithm>
#include <cmath>

#include <QPainter>

#include <tobas_std_tools/unit_conversions.hpp>

namespace tobas
{
namespace gui
{
namespace sc
{
namespace
{
void drawMarker(QPainter& painter, const QPointF& center, double radius, Qt::GlobalColor global_color)
{
  constexpr int kFillAlpha = 70;
  constexpr int kFillLightness = 150;
  constexpr double kLineWidth = 3.0;

  const QColor color(global_color);
  auto fill_color = color.lighter(kFillLightness);
  fill_color.setAlpha(kFillAlpha);

  painter.save();
  painter.setPen(QPen(color, kLineWidth));
  painter.setBrush(fill_color);
  painter.drawEllipse(center, radius, radius);
  painter.drawLine(QPointF(center.x() - radius, center.y()), QPointF(center.x() + radius, center.y()));
  painter.drawLine(QPointF(center.x(), center.y() - radius), QPointF(center.x(), center.y() + radius));
  painter.restore();
}

QFont displayFont(const QFont& base, int point_size)
{
  auto font = base;
  font.setPointSize(point_size);
  font.setBold(true);
  return font;
}

std::pair<QPointF, double>
computeIndicatorPosition(const kdl::Vector& acc, const QPointF& center, double radius, double marker_radius)
{
  constexpr double kMaxTilt = st::deg2rad(10.0);  // [rad]
  constexpr double kBoundaryPadding = 3.0;

  const auto horizontal_acc = std::hypot(acc.x(), acc.y());
  const auto tilt = std::atan2(horizontal_acc, acc.z());
  const auto max_distance = std::max(0.0, radius - marker_radius - kBoundaryPadding);
  const auto distance = std::clamp(tilt / kMaxTilt, 0.0, 1.0) * max_distance;

  auto marker_center = center;
  if (horizontal_acc > 0.0) {
    // The widget is a top view: +X is front and +Y is left.
    marker_center.rx() -= acc.y() / horizontal_acc * distance;
    marker_center.ry() -= acc.x() / horizontal_acc * distance;
  }

  return { marker_center, tilt };
}
}  // namespace

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

void LevelIndicatorWidget::paintEvent(QPaintEvent*)
{
  constexpr int kOuterMargin = 48;
  constexpr int kDirectionFontPointSize = 16;
  constexpr int kReadingFontPointSize = 12;
  constexpr int kReadingLineHeight = 26;
  constexpr int kReadingBottomMargin = 6;
  constexpr int kTiltPrecision = 1;
  constexpr double kOutlineWidth = 2.0;
  constexpr double kGuideWidth = 1.0;
  constexpr double kMinimumMarkerRadius = 9.0;
  constexpr double kMarkerRadiusRatio = 1.0 / 16.0;
  constexpr double kTargetRadiusRatio = 0.1;
  constexpr auto kRawColor = Qt::darkYellow;
  constexpr auto kCalibratedColor = Qt::darkGreen;

  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);

  const QPointF center(width() / 2.0, height() / 2.0);
  const auto radius = std::max(0.0, std::min(width(), height()) / 2.0 - kOuterMargin);
  const QRectF level_rect(center.x() - radius, center.y() - radius, 2.0 * radius, 2.0 * radius);

  // Draw the level background and outline.
  painter.setPen(QPen(palette().color(QPalette::Mid), kOutlineWidth));
  painter.setBrush(palette().color(QPalette::Base));
  painter.drawEllipse(level_rect);

  // Draw the crosshair and concentric tilt guides.
  painter.setPen(QPen(palette().color(QPalette::Mid), kGuideWidth, Qt::DashLine));
  painter.drawLine(QPointF(center.x() - radius, center.y()), QPointF(center.x() + radius, center.y()));
  painter.drawLine(QPointF(center.x(), center.y() - radius), QPointF(center.x(), center.y() + radius));
  painter.drawEllipse(center, radius / 2.0, radius / 2.0);

  const auto target_radius = radius * kTargetRadiusRatio;
  painter.setPen(QPen(palette().color(QPalette::Mid), kOutlineWidth));
  painter.drawEllipse(center, target_radius, target_radius);

  // Draw orientation labels for the FLU body frame.
  painter.setFont(displayFont(font(), kDirectionFontPointSize));
  painter.setPen(palette().color(QPalette::Text));
  painter.drawText(
    QRectF(level_rect.left(), level_rect.top() - kOuterMargin, level_rect.width(), kOuterMargin),
    Qt::AlignCenter,
    "Front");
  painter.drawText(
    QRectF(level_rect.left(), level_rect.bottom(), level_rect.width(), kOuterMargin), Qt::AlignCenter, "Rear");
  painter.drawText(
    QRectF(level_rect.left() - kOuterMargin, level_rect.top(), kOuterMargin, level_rect.height()),
    Qt::AlignCenter,
    "Left");
  painter.drawText(
    QRectF(level_rect.right(), level_rect.top(), kOuterMargin, level_rect.height()), Qt::AlignCenter, "Right");

  const auto marker_radius = std::max(kMinimumMarkerRadius, radius * kMarkerRadiusRatio);
  const auto raw_text_rect = QRectF(
    level_rect.left(),
    level_rect.bottom() - 2 * kReadingLineHeight - kReadingBottomMargin,
    level_rect.width(),
    kReadingLineHeight);
  const auto calib_text_rect = raw_text_rect.translated(0.0, static_cast<double>(kReadingLineHeight));
  painter.setFont(displayFont(font(), kReadingFontPointSize));

  if (has_raw_data_) {
    const auto& acc_raw = acc_raw_lpf_.getValue();
    const auto [raw_center, raw_tilt] = computeIndicatorPosition(acc_raw, center, radius, marker_radius);
    drawMarker(painter, raw_center, marker_radius, kRawColor);

    const auto raw_text = QString("Raw:             %1 deg").arg(st::rad2deg(raw_tilt), 0, 'f', kTiltPrecision);
    painter.setPen(QColor(kRawColor));
    painter.drawText(raw_text_rect, Qt::AlignCenter, raw_text);
  }
  else {
    painter.setPen(QColor(kRawColor));
    painter.drawText(raw_text_rect, Qt::AlignCenter, "Raw:             Waiting...");
  }

  if (has_calib_data_) {
    const auto& acc_calib = acc_calib_lpf_.getValue();
    const auto [calib_center, calib_tilt] = computeIndicatorPosition(acc_calib, center, radius, marker_radius);
    drawMarker(painter, calib_center, marker_radius, kCalibratedColor);

    const auto calib_text = QString("Calibrated: %1 deg").arg(st::rad2deg(calib_tilt), 0, 'f', kTiltPrecision);
    painter.setPen(QColor(kCalibratedColor));
    painter.drawText(calib_text_rect, Qt::AlignCenter, calib_text);
  }
  else {
    painter.setPen(QColor(kCalibratedColor));
    painter.drawText(calib_text_rect, Qt::AlignCenter, "Calibrated: Waiting...");
  }
}
}  // namespace sc
}  // namespace gui
}  // namespace tobas
