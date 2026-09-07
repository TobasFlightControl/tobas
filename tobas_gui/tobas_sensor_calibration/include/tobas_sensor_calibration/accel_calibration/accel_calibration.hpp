// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QPushButton>

#include <tobas_qt_tools/widgets/wait_spinner.hpp>
#include <tobas_ros2_tools/register.hpp>

#include "../base.hpp"
#include "./level_indicator.hpp"
#include "./thread.hpp"

namespace tobas
{
namespace gui
{
namespace sc
{
class AccelCalibrationWidget : public BaseWidget
{
  Q_OBJECT

  using self = AccelCalibrationWidget;
  using super = BaseWidget;

public:
  explicit AccelCalibrationWidget(const rqt::RosQtBridge& bridge);

  const char* title() const override;
  void reset() override;
  void updateInternalDataStructures() override;
  void initializeRosInterfaces(rclcpp::Node::SharedPtr node, const std::string& ns) override;
  void clearRosInterfaces() override;

private:
  QPushButton* start_button_;
  LevelIndicatorWidget* level_indicator_;

  qt::WaitSpinnerWidget spinner_;
  AccelCalibrationThread thread_;

  tobas_msgs::msg::Arming::ConstSharedPtr arming_;
  tobas_msgs::Imu::ConstSharedPtr imu_raw_, imu_calib_;

private Q_SLOTS:
  void onStartButtonClicked();

  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming);
  void rawImuCb(const tobas_msgs::Imu::ConstSharedPtr& imu_raw);
  void calibratedImuCb(const tobas_msgs::Imu::ConstSharedPtr& imu_calib);
};
}  // namespace sc
}  // namespace gui
}  // namespace tobas
