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

public:
  explicit SensorCalibrationWidget(const rqt::RosQtBridge& _bridge, const Drone& _drone);

  void reset();
  void updateInternalDataStructures();
  void clearRosInterfaces();
  void initializeRosInterfaces(rclcpp::Node::SharedPtr _node, const std::string& _ns);

private:
  const Drone& drone_;

  AccelCalibrationWidget* accel_calib_;
  MagCalibrationWidget* mag_calib_;
  RCInputCalibrationWidget* rcin_calib_;

  BaseWidget* getWidget(int _index);
  const BaseWidget* getWidget(int _index) const;

  void setTabsEnabled(bool _enabled);

  void setCompleted(int _index);
  void setCompleted(BaseWidget* _widget);
  void setIncompleted(int _index);
  void setIncompleted(BaseWidget* _widget);

private Q_SLOTS:
  void imuCb(const tobas_msgs::Imu::ConstSharedPtr& _msg);
  void magCb(const tobas_msgs::MagneticField::ConstSharedPtr& _msg);
  void rcInputCb(const tobas_msgs::RCInput::ConstSharedPtr& _msg);
};
}  // namespace sc
}  // namespace gui
}  // namespace tobas
