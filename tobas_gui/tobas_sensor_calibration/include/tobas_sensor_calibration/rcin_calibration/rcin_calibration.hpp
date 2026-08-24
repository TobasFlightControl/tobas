// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QPushButton>

#include <tobas_constants/rc_input.hpp>
#include <tobas_drone_core/drone.hpp>
#include <tobas_qt_tools/widgets/position_bar_widget.hpp>
#include <tobas_ros2_tools/sync_service_client.hpp>
#include <tobas_rqt_bridge/bridge.hpp>

#include <tobas_real_msgs/srv/set_rc_input_params.hpp>

#include "../base.hpp"

namespace tobas
{
namespace gui
{

namespace sc
{
class RCInputCalibrationWidget : public BaseWidget
{
  Q_OBJECT

  using self = RCInputCalibrationWidget;
  using super = BaseWidget;

  // The basic S.BUS throttle range is 172 to 1811.
  static constexpr int kMinPeriod = 0;
  static constexpr int kMaxPeriod = 2000;

  // Each S.BUS channel value is at least 1000 and at most 2000, so
  // using 1000 as the threshold prevents the human error of moving a three-position switch through only two positions.
  static constexpr int kMinSignalRange = 1000;

  static constexpr char kOnText[] = "ON";
  static constexpr char kOffText[] = "OFF";

  static constexpr int kButtonWidth = 100;
  static constexpr int kButtonHeight = 40;
  static constexpr int kRangeSideShort = 50;

public:
  explicit RCInputCalibrationWidget(const RosQtBridge& bridge, const Drone& drone);

  const char* title() const override;

  void reset() override;

  void updateInternalDataStructures();
  void initializeRosInterfaces(rclcpp::Node::SharedPtr node, const std::string& ns);

private:
  const Drone& drone_;

  ros2::SyncServiceClient<tobas_real_msgs::srv::SetRcInputParams>::SharedPtr set_params_sc_;

  bool ros_initialized_ = false;
  bool running_ = false;
  tobas_msgs::msg::Sbus::ConstSharedPtr sbus_;
  tobas_msgs::msg::Arming::ConstSharedPtr arming_;

  QPushButton* start_button_;
  QPushButton* finish_button_;
  QPushButton* cancel_button_;

  qt::HPositionBarWidget* roll_range_;
  qt::VPositionBarWidget* pitch_range_;
  qt::HPositionBarWidget* yaw_range_;
  qt::VPositionBarWidget* throt_range_;
  qt::HPositionBarWidget* mode_range_;
  qt::HPositionBarWidget* sub_mode_range_;
  qt::HPositionBarWidget* enable_range_;
  qt::HPositionBarWidget* kill_range_;

  std::array<QLabel*, kMaxNumOfGpsw> gpsw_labels_;
  std::array<qt::HPositionBarWidget*, kMaxNumOfGpsw> gpsw_ranges_;

  size_t numOfGpswChannels() const;

  bool saveParamsToGcs();
  bool saveParamsToFc();

private Q_SLOTS:
  void onStartButtonClicked();
  void onCancelButtonClicked();
  void onFinishButtonClicked();

  void sbusCb(const tobas_msgs::msg::Sbus::ConstSharedPtr& sbus);
  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming);
};
}  // namespace sc
}  // namespace gui
}  // namespace tobas
