// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QPushButton>

#include <tobas_qt_tools/widgets/wait_spinner.hpp>
#include <tobas_rqt_bridge/bridge.hpp>

#include "../base.hpp"
#include "./thread.hpp"

namespace tobas
{
namespace gui
{
namespace sc
{
class LargeVehicleMagCalibWidget : public BaseMagCalibWidget
{
  Q_OBJECT

  using self = LargeVehicleMagCalibWidget;

public:
  explicit LargeVehicleMagCalibWidget(const rqt::RosQtBridge& _bridge);

  void reset() override;
  void initializeRosInterfaces(rclcpp::Node::SharedPtr _node, const std::string& _ns) override;
  void clearRosInterfaces() override;

private:
  QPushButton* start_button_;

  qt::WaitSpinnerWidget spinner_;
  LargeVehicleMagCalibThread thread_;

  tobas_msgs::msg::Arming::ConstSharedPtr arming_;

private Q_SLOTS:
  void onStartButtonClicked();

  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& _arming);
};
}  // namespace sc
}  // namespace gui
}  // namespace tobas
