// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_qt_tools/widgets/widget.hpp>
#include <tobas_rqt_bridge/bridge.hpp>

namespace tobas::gui::ctrl
{
class PoseViewerWidget : public qt::Widget
{
  Q_OBJECT

  using self = PoseViewerWidget;

public:
  explicit PoseViewerWidget(const rqt::RosQtBridge& _bridge);

  void reset();

private:
  double roll_, pitch_, yaw_;  ///< Current Euler angles.
  double alt_rel_;             ///< Current relative altitude from the takeoff point [m].
  double alt_msl_;             ///< Current altitude above mean sea level [m].
  bool alt_msl_valid_;

  void paintEvent(QPaintEvent* _event) override;

  void scale(QPainter& _painter, bool _keep_aspect);

  void drawGround(QPainter& _painter);
  void drawSky(QPainter& _painter);
  void drawRoll(QPainter& _painter);
  void drawPitch(QPainter& _painter);
  void drawYaw(QPainter& _painter);
  void drawRelativeAltitude(QPainter& _painter);
  void drawMslAltitude(QPainter& _painter);

  void addGradation(QPainter& _painter);

  /** Determine whether a point in the camera frame is included in the sky. */
  bool isSky(const QPoint& _p, double _a, double _b) const;

  /** Convert pitch angle [rad] to window height. */
  static double pitchToHeight(double _pitch);

  /** Convert yaw angle [rad] to window width. */
  static double yawToWidth(double _yaw);

  /** Convert altitude difference [m] to window height. */
  static double altitudeToHeight(double _altitude);

private Q_SLOTS:
  void odomCb(const tobas_msgs::OdometryWithCovarianceStamped::ConstSharedPtr& _odom);
  void gnssCb(const tobas_msgs::Gnss::ConstSharedPtr& _gnss);
};
}  // namespace tobas::gui::ctrl
