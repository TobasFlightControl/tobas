// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_qt_tools/widgets/widget.hpp>
#include <tobas_rqt_bridge/bridge.hpp>

namespace tobas
{
namespace gui
{
namespace ctrl
{
class PoseViewerWidget : public qt::Widget
{
  Q_OBJECT

  using self = PoseViewerWidget;
  using super = qt::Widget;

public:
  explicit PoseViewerWidget(const rqt::RosQtBridge& bridge);

  void reset();

private:
  double roll_, pitch_, yaw_;  // Current Euler angles.
  double alt_rel_;             // Current relative altitude from the takeoff point [m].
  double alt_msl_;             // Current altitude above mean sea level [m].
  bool alt_msl_valid_;

  void paintEvent(QPaintEvent* event) override;

  void scale(QPainter& painter, bool keep_aspect);

  void drawGround(QPainter& painter);
  void drawSky(QPainter& painter);
  void drawRoll(QPainter& painter);
  void drawPitch(QPainter& painter);
  void drawYaw(QPainter& painter);
  void drawRelativeAltitude(QPainter& painter);
  void drawMslAltitude(QPainter& painter);

  void addGradation(QPainter& painter);

  /* Determine whether a point in the camera frame is included in the sky. */
  bool isSky(const QPoint& p, double a, double b) const;

  /* Convert pitch angle [rad] to window height. */
  static double pitchToHeight(double pitch);

  /* Convert yaw angle [rad] to window width. */
  static double yawToWidth(double yaw);

  /* Convert altitude difference [m] to window height. */
  static double altitudeToHeight(double altitude);

private Q_SLOTS:
  void odomCb(const tobas_msgs::OdometryWithCovarianceStamped::ConstSharedPtr& odom);
  void gnssCb(const tobas_msgs::Gnss::ConstSharedPtr& gnss);
};
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
