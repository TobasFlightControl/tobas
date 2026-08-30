// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <tobas_qt_tools/widgets/vertical_tab_widget.hpp>

#include "./accel_calibration/accel_calibration.hpp"
#include "./mag_calibration/mag_calibration.hpp"
#include "./rcin_calibration/rcin_calibration.hpp"

namespace tobas
{
namespace gui
{
namespace sc
{
class SensorCalibrationWidget : public qt::VerticalTabWidget
{
  Q_OBJECT

  using self = SensorCalibrationWidget;
  using super = qt::VerticalTabWidget;

  // Without at least this much height, the `TabBar` text is clipped horizontally for some reason.
  static constexpr int kTabHeight = 35;
  static constexpr int kTabWidth = 70;

public:
  explicit SensorCalibrationWidget(const rqt::RosQtBridge& bridge, const Drone& drone);

  void reset();
  void updateInternalDataStructures();
  void clearRosInterfaces();
  void initializeRosInterfaces(rclcpp::Node::SharedPtr node, const std::string& ns);

private:
  const Drone& drone_;

  AccelCalibrationWidget* accel_calib_;
  MagCalibrationWidget* mag_calib_;
  RCInputCalibrationWidget* rcin_calib_;

  BaseWidget* getWidget(int index);
  const BaseWidget* getWidget(int index) const;

  void setTabsEnabled(bool enabled);

  void setCompleted(int index);
  void setCompleted(BaseWidget* widget);
  void setIncompleted(int index);
  void setIncompleted(BaseWidget* widget);

private Q_SLOTS:
  void imuCb(const tobas_msgs::Imu::ConstSharedPtr& msg);
  void magCb(const tobas_msgs::MagneticField::ConstSharedPtr& msg);
  void rcInputCb(const tobas_msgs::RCInput::ConstSharedPtr& msg);
};
}  // namespace sc
}  // namespace gui
}  // namespace tobas
