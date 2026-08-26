// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_control_system/pose_viewer.hpp"

#include <QDebug>
#include <QPainter>

#include <tobas_math/core.hpp>

namespace tobas
{
namespace gui
{
namespace ctrl
{
PoseViewerWidget::PoseViewerWidget(const rqt::RosQtBridge& bridge)
{
  reset();

  connect(&bridge, &rqt::RosQtBridge::odomReceived, this, &self::odomCb, Qt::QueuedConnection);
  connect(&bridge, &rqt::RosQtBridge::gnssReceived, this, &self::gnssCb, Qt::QueuedConnection);
}

void PoseViewerWidget::reset()
{
  roll_ = 0.0;
  pitch_ = 0.0;
  yaw_ = 0.0;
  alt_rel_ = 0.0;
  alt_msl_ = 0.0;
  alt_msl_valid_ = false;

  update();
}

void PoseViewerWidget::paintEvent(QPaintEvent*)
{
  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing);

  drawGround(painter);
  drawSky(painter);

  addGradation(painter);

  drawRoll(painter);
  drawPitch(painter);
  drawYaw(painter);
  drawRelativeAltitude(painter);
  drawMslAltitude(painter);
}

void PoseViewerWidget::scale(QPainter& painter, bool keep_aspect)
{
  if (keep_aspect) {
    const auto scale = static_cast<double>(std::min(width(), height())) / kOriginalSize;
    painter.scale(scale, scale);
  }
  else {
    const auto x_scale = static_cast<double>(width()) / kOriginalSize;
    const auto y_scale = static_cast<double>(height()) / kOriginalSize;
    painter.scale(x_scale, y_scale);
  }
}

void PoseViewerWidget::drawGround(QPainter& painter)
{
  painter.fillRect(rect(), QColor(169, 110, 45));
}

void PoseViewerWidget::drawSky(QPainter& painter)
{
  const auto w = width();
  const auto h = height();

  // Four corner points of the drawing area.
  const QPoint OO(0, 0);
  const QPoint WO(w, 0);
  const QPoint OH(0, h);
  const QPoint WH(w, h);

  // Pitch angle ratio relative to the vertical angle of view.
  const auto r = pitch_ / (kPitchAngleOfView / 2);

  // Create points that make up the sky area (memo: 3-43).
  QVector<QPoint> points;
  if (roll_ == 0.0) {
    const auto y = (h / 2) * (1 - r);
    if (0.0 <= y && y <= h) {
      const QPoint OY(0, y);
      const QPoint WY(w, y);
      points = { OO, WO, WY, OY };
    }
    else {
      points = {};
    }
  }
  else {
    // Line equation: y = ax + b.
    const auto sin_phi = std::sin(roll_);
    const auto cos_phi = std::cos(roll_);
    const auto tan_phi = std::tan(roll_);
    const auto a = -tan_phi;
    const auto b = (tan_phi / 2) * (w - h * r * sin_phi) + (h / 2) * (1 - r * cos_phi);

    // Intersections between the line and drawing-area edges.
    const QPoint XO(-b / a, 0);       // Intersection with y = 0.
    const QPoint XH((h - b) / a, h);  // Intersection with y = h.
    const QPoint OY(0, b);            // Intersection with x = 0.
    const QPoint WY(w, a * w + b);    // Intersection with x = w.

    // Determine whether each corner of the drawing area is included in the sky area.
    const auto OO_sky = isSky(OO, a, b);
    const auto WO_sky = isSky(WO, a, b);
    const auto OH_sky = isSky(OH, a, b);
    const auto WH_sky = isSky(WH, a, b);

    if (!OO_sky && !WO_sky && !OH_sky && !WH_sky)  // 0000
    {
      points = {};
    }
    else if (!OO_sky && !WO_sky && !OH_sky && WH_sky)  // 0001
    {
      points = { WH, XH, WY };
    }
    else if (!OO_sky && !WO_sky && OH_sky && !WH_sky)  // 0010
    {
      points = { OH, XH, OY };
    }
    else if (!OO_sky && !WO_sky && OH_sky && WH_sky)  // 0011
    {
      points = { OH, WH, WY, OY };
    }
    else if (!OO_sky && WO_sky && !OH_sky && !WH_sky)  // 0100
    {
      points = { WO, XO, WY };
    }
    else if (!OO_sky && WO_sky && !OH_sky && WH_sky)  // 0101
    {
      points = { WO, WH, XH, XO };
    }
    else if (!OO_sky && WO_sky && OH_sky && WH_sky)  // 0111
    {
      points = { WO, WH, OH, OY, XO };
    }
    else if (OO_sky && !WO_sky && !OH_sky && !WH_sky)  // 1000
    {
      points = { OO, XO, OY };
    }
    else if (OO_sky && !WO_sky && OH_sky && !WH_sky)  // 1010
    {
      points = { OO, OH, XH, XO };
    }
    else if (OO_sky && !WO_sky && OH_sky && WH_sky)  // 1011
    {
      points = { OO, OH, WH, WY, XO };
    }
    else if (OO_sky && WO_sky && !OH_sky && !WH_sky)  // 1100
    {
      points = { OO, WO, WY, OY };
    }
    else if (OO_sky && WO_sky && !OH_sky && WH_sky)  // 1101
    {
      points = { OO, WO, WH, XH, OY };
    }
    else if (OO_sky && WO_sky && OH_sky && !WH_sky)  // 1110
    {
      points = { WO, OO, OH, XH, WY };
    }
    else if (OO_sky && WO_sky && OH_sky && WH_sky)  // 1111
    {
      points = { OO, WO, WH, OH };
    }
    else {
      qWarning() << "Impossible ground-sky pattern.";
      return;
    }
  }

  // Draw.
  QPolygon polygon(points);
  painter.save();
  painter.setBrush(QColor(36, 139, 255));
  painter.drawPolygon(polygon);
  painter.restore();
}

void PoseViewerWidget::drawRoll(QPainter& painter)
{
  painter.save();

  // Move to the circle center as seen from the vehicle.
  painter.translate(width() / 2, height() / 2);
  painter.rotate(-st::rad2deg(roll_));

  // Scale to the widget size.
  scale(painter, true);

  // Draw the circle.
  painter.setPen(QPen(Qt::white, kLineWidth));
  painter.drawEllipse(QPoint(0, 0), kRollRadius, kRollRadius);

  // Draw each value.
  const auto outer_radius = kRollRadius + kRollTickLength;
  const auto text_radius = outer_radius + 20;
  for (int deg = 0; deg < 360; deg += kScaleInterval) {
    // Draw ticks.
    painter.drawLine(0, -kRollRadius, 0, -outer_radius);

    // Draw numbers.
    painter.drawText(-10, -text_radius, std::format("{}°", math::wrap(deg, 180)).c_str());

    // Advance by the tick interval.
    painter.rotate(kScaleInterval);
  }

  painter.restore();

  // Draw a marker at the current position.
  painter.save();
  painter.translate(width() / 2, height() / 2);
  scale(painter, true);
  painter.setPen(QPen(Qt::red, kLineWidth));
  painter.drawLine(0, -kRollRadius - kRollTickLength * 2, 0, -kRollRadius);
  painter.restore();
}

void PoseViewerWidget::drawPitch(QPainter& painter)
{
  painter.save();

  // Move to the center position as seen from the vehicle.
  painter.translate(width() / 2, height() / 2);
  painter.rotate(-st::rad2deg(roll_));

  // Scale to the widget size.
  scale(painter, true);

  // Determine the value range to draw.
  const auto pitch_deg = st::rad2deg(pitch_);
  const auto pitch_min = math::floor(pitch_deg - kPitchVisualRange, kScaleInterval);
  const auto pitch_max = math::ceil(pitch_deg + kPitchVisualRange, kScaleInterval);

  // Move to the initial position.
  painter.translate(0, pitchToHeight(st::deg2rad(pitch_min - pitch_deg)));

  // Draw each value.
  const auto line_half = kPitchLineLength / 2;
  const auto text_x = -line_half - 30;
  const auto text_y = 5;
  const auto y_interval = pitchToHeight(st::deg2rad(kScaleInterval));
  painter.setPen(QPen(Qt::white, kLineWidth));
  for (int deg = pitch_min; deg <= pitch_max; deg += kScaleInterval) {
    // Draw ticks.
    painter.drawLine(-line_half, 0, line_half, 0);

    // Draw numbers.
    painter.drawText(text_x, text_y, std::format("{}°", math::wrap(deg, 180)).c_str());

    // Advance by the tick interval.
    painter.translate(0, y_interval);
  }

  painter.restore();

  // Draw a marker at the current position.
  painter.save();
  painter.translate(width() / 2, height() / 2);
  painter.rotate(-st::rad2deg(roll_));
  scale(painter, true);
  painter.setPen(QPen(Qt::red, kLineWidth));
  painter.drawLine(-line_half, 0, line_half, 0);
  painter.restore();
}

void PoseViewerWidget::drawYaw(QPainter& painter)
{
  painter.save();

  // Scale to the widget size.
  scale(painter, false);

  // Move to the center position.
  const auto beta = kYawAngleOfView / 2;  // [rad]
  painter.translate(yawToWidth(beta), kYawLineY);

  // Draw the number line.
  painter.setPen(QPen(Qt::white, kLineWidth));
  painter.drawLine(-kOriginalSize / 2, 0, kOriginalSize / 2, 0);

  // Determine the value range to draw.
  const auto yaw_deg = st::rad2deg(yaw_);
  const auto yaw_min = math::floor(st::rad2deg(yaw_ - beta), kScaleInterval);
  const auto yaw_max = math::ceil(st::rad2deg(yaw_ + beta), kScaleInterval);

  // Move to the initial position.
  painter.translate(yawToWidth(st::deg2rad(yaw_deg - yaw_min)), 0);

  // Draw each value.
  const auto text_x = -10;
  const auto text_y = -kYawTickLength - 20;
  const auto x_interval = yawToWidth(st::deg2rad(kScaleInterval));
  for (int deg = yaw_min; deg <= yaw_max; deg += kScaleInterval) {
    // Draw ticks.
    painter.drawLine(0, 0, 0, -kYawTickLength);

    // Draw numbers.
    painter.drawText(text_x, text_y, std::format("{}°", math::wrap(deg, 180)).c_str());

    // Advance by the tick interval.
    painter.translate(-x_interval, 0);
  }

  painter.restore();

  // Draw a marker at the current position.
  painter.save();
  scale(painter, false);
  painter.translate(yawToWidth(beta), kYawLineY);
  painter.setPen(QPen(Qt::red, kLineWidth));
  painter.drawLine(0, 0, 0, -kYawTickLength * 2);
  painter.restore();
}

void PoseViewerWidget::drawRelativeAltitude(QPainter& painter)
{
  painter.save();

  scale(painter, false);

  const auto center_y = kOriginalSize / 2;
  const auto alt_min = math::floor(alt_rel_ - kAltitudeVisualRange, kAltitudeScaleInterval);
  const auto alt_max = math::ceil(alt_rel_ + kAltitudeVisualRange, kAltitudeScaleInterval);
  const auto x = kAltitudeTickX;

  painter.setPen(QPen(Qt::white, kLineWidth));
  painter.drawText(x - 2, kAltitudeTextY, "REL ALT");

  for (int alt = alt_min; alt <= alt_max; alt += kAltitudeScaleInterval) {
    const auto y = center_y - altitudeToHeight(alt - alt_rel_);
    if (y <= kAltitudeTickMaxY) {
      continue;
    }

    painter.drawLine(x, y, x + kAltitudeTickLength, y);
    painter.drawText(x + kAltitudeTickLength + 10, y + 5, std::format("{}m", alt).c_str());
  }

  painter.setPen(QPen(Qt::red, kLineWidth));
  painter.drawLine(x - kAltitudeTickLength, center_y, x + kAltitudeTickLength * 2, center_y);
  painter.drawText(x + kAltitudeTickLength + 10, center_y - 12, std::format("{:.1f}m", alt_rel_).c_str());

  painter.restore();
}

void PoseViewerWidget::drawMslAltitude(QPainter& painter)
{
  painter.save();

  scale(painter, false);

  const auto center_y = kOriginalSize / 2;
  const auto alt_min = math::floor(alt_msl_ - kAltitudeVisualRange, kAltitudeScaleInterval);
  const auto alt_max = math::ceil(alt_msl_ + kAltitudeVisualRange, kAltitudeScaleInterval);
  const auto x = kOriginalSize - kAltitudeTickX;

  painter.setPen(QPen(Qt::white, kLineWidth));
  painter.drawText(x - 55, kAltitudeTextY, "MSL ALT");

  if (alt_msl_valid_) {
    for (int alt = alt_min; alt <= alt_max; alt += kAltitudeScaleInterval) {
      const auto y = center_y - altitudeToHeight(alt - alt_msl_);
      if (y <= kAltitudeTickMaxY) {
        continue;
      }

      painter.drawLine(x, y, x - kAltitudeTickLength, y);
      painter.drawText(x - kAltitudeTickLength - 45, y + 5, std::format("{}m", alt).c_str());
    }
  }

  painter.setPen(QPen(Qt::red, kLineWidth));
  painter.drawLine(x + kAltitudeTickLength, center_y, x - kAltitudeTickLength * 2, center_y);

  const QString text = alt_msl_valid_ ? std::format("{:.1f}m", alt_msl_).c_str() : "---m";
  painter.drawText(x - kAltitudeTickLength - 45, center_y - 12, text);

  painter.restore();
}

void PoseViewerWidget::addGradation(QPainter& painter)
{
  painter.save();

  // Gradient settings to brighten the center.
  QRadialGradient grad(getCenter(), width() / 2);

  grad.setColorAt(0, QColor(255, 255, 255, 100));  // The center is translucent white.
  grad.setColorAt(1, QColor(255, 255, 255, 0));    // The outside is transparent.

  // Set the gradient on the brush.
  painter.setBrush(grad);
  painter.setPen(Qt::NoPen);  // Fill without a pen.

  // Overlay brightness.
  painter.drawRect(0, 0, width(), height());  // Apply a radial gradient to the entire widget.

  painter.restore();
}

bool PoseViewerWidget::isSky(const QPoint& p, double a, double b) const
{
  const auto left = p.y();
  const auto right = a * p.x() + b;

  // Branch by roll angle. If the roll angle exceeds 90 degrees, up and down are inverted.
  if (std::abs(roll_) < M_PI_2) {
    return left < right;
  }
  else {
    return left > right;
  }
}

double PoseViewerWidget::pitchToHeight(double pitch)
{
  return kOriginalSize * pitch / kPitchAngleOfView;
}

double PoseViewerWidget::yawToWidth(double yaw)
{
  return kOriginalSize * yaw / kYawAngleOfView;
}

double PoseViewerWidget::altitudeToHeight(double altitude)
{
  return (kOriginalSize / 2) * altitude / kAltitudeVisualRange;
}

void PoseViewerWidget::odomCb(const tobas_msgs::OdometryWithCovarianceStamped::ConstSharedPtr& odom)
{
  odom->odom.odom.frame.M.getRPY(roll_, pitch_, yaw_);
  alt_rel_ = odom->odom.odom.frame.p.z();

  update();
}

void PoseViewerWidget::gnssCb(const tobas_msgs::Gnss::ConstSharedPtr& gnss)
{
  alt_msl_ = gnss->height_msl;
  alt_msl_valid_ = (gnss->fix_type == tobas_msgs::msg::Gnss::FIX_3D);

  update();
}
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
