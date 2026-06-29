// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_qt_tools/widgets/widget.hpp>
#include <tobas_rqt_bridge/bridge.hpp>
#include <tobas_std_tools/unit_conversions.hpp>

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

  static constexpr int kOriginalSize = 640;
  static constexpr int kLineWidth = 3;       // ゲージ線の幅
  static constexpr int kScaleInterval = 10;  // [deg]
  static constexpr int kRollRadius = 200;    // ロール円の半径
  static constexpr int kRollTickLength = 10;
  static constexpr double kPitchAngleOfView = st::deg2rad(120);  // [rad] 人間の視野角程度
  static constexpr int kPitchVisualRange = 25;                   // [deg] 描画するピッチ角の範囲
  static constexpr int kPitchLineLength = 100;
  static constexpr double kYawAngleOfView = st::deg2rad(120);  // [rad]
  static constexpr int kYawLineY = 60;
  static constexpr int kYawTickLength = 10;
  static constexpr int kAltitudeVisualRange = 25;   // [m]
  static constexpr int kAltitudeScaleInterval = 5;  // [m]
  static constexpr int kAltitudeTickLength = 12;
  static constexpr int kAltitudeTickX = 40;
  static constexpr int kAltitudeTextY = kYawLineY + 40;
  static constexpr int kAltitudeTickMaxY = kAltitudeTextY + 10;

public:
  explicit PoseViewerWidget(const RosQtBridge& bridge);

  void reset();

private:
  double roll_, pitch_, yaw_;  // 現在のオイラー角
  double alt_rel_;             // 現在の起動地点からの相対高度 [m]
  double alt_msl_;             // 現在の海抜高度 [m]
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

  /* カメラの枠内の点が空に含まれるかどうかを判定する． */
  bool isSky(const QPoint& p, double a, double b) const;

  /* ピッチ角 [rad] をウィンドウ高さに変換する． */
  static double pitchToHeight(double pitch);

  /* ヨー角 [rad] をウィンドウ幅に変換する． */
  static double yawToWidth(double yaw);

  /* 高度差 [m] をウィンドウ高さに変換する． */
  static double altitudeToHeight(double altitude);

private Q_SLOTS:
  void odomCb(const tobas_msgs::OdometryWithCovarianceStamped::ConstSharedPtr& odom);
  void gnssCb(const tobas_msgs::Gnss::ConstSharedPtr& gnss);
};
}  // namespace ctrl
}  // namespace gui
}  // namespace tobas
