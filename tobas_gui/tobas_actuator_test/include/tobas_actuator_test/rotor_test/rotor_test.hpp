// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QPushButton>
#include <QTimer>

#include <tobas_drone_core/drone.hpp>
#include <tobas_drone_core/propulsion_system/electric_propulsion_system/electric_propulsion_system.hpp>
#include <tobas_gui_common/project_paths.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_ros2_tools/register.hpp>
#include <tobas_ros2_tools/sync_service_client.hpp>
#include <tobas_rqt_bridge/bridge.hpp>

#include <tobas_constants/hardware.hpp>
#include <tobas_dparam_client/dparam_client.hpp>

#include <tobas_dparam_msgs/srv/get_params.hpp>
#include <tobas_msgs/msg/arming.hpp>
#include <tobas_msgs/msg/rotor_speed_array.hpp>
#include <tobas_msgs/msg/rotor_state_array.hpp>

#include "../base.hpp"
#include "./rotor_widget.hpp"

namespace tobas
{
namespace gui
{
namespace at
{
class RotorTestWidget : public BaseWidget
{
  Q_OBJECT

  using self = RotorTestWidget;
  using super = BaseWidget;

  static constexpr int kButtonWidth = 100;
  static constexpr int kButtonHeight = 40;
  static constexpr int kUpdatePeriod = 10;  // [ms]
  static constexpr auto kWaitForService = std::chrono::seconds(3);

public:
  explicit RotorTestWidget(const RosQtBridge& bridge, const Drone& drone);

  const char* title() const override;

  void reset() override;

  void updateProject(const std::filesystem::path& proj_path);
  void initializeRosInterfaces(rclcpp::Node::SharedPtr node, const std::string& ns);

  int numRegisteredChannels() const;

private:
  rclcpp::Node::SharedPtr node_;
  const RosQtBridge& bridge_;
  const Drone& drone_;
  ElectricPropulsionSystemConfig::ConstSharedPtr eprop_;
  cmn::ProjectPaths proj_paths_;

  QPushButton* start_button_;
  QPushButton* stop_button_;
  QPushButton* save_button_;

  std::array<bool, kMaxDshotChannels> registered_;
  std::array<RotorWidget*, kMaxDshotChannels> rotor_widgets_;

  bool ros_initialized_ = false;
  bool running_ = false;
  tobas_msgs::msg::Arming::ConstSharedPtr arming_;

  ros2::PublisherPtr<tobas_msgs::msg::RotorSpeedArray> tar_speeds_pub_;
  dparam::DynamicParamClient::SharedPtr dparam_cli_;
  ros2::SyncServiceClient<tobas_dparam_msgs::srv::GetParams>::SharedPtr get_params_sc_;

  QMetaObject::Connection rotor_states_conn_;

  QTimer update_timer_;

  void publishTargetSppeds();
  bool loadCurrentGains();

private Q_SLOTS:
  void onStartButtonClicked();
  void onStopButtonClicked();
  void onSaveButtonClicked();

  void onTargetRPMChanged(int rpm, size_t ch);
  void onGainChanged(int gain, size_t ch);

  void onUpdateTimerTimeout();

  void rotorStatesCb(const tobas_msgs::msg::RotorStateArray::ConstSharedPtr& cur_states);
  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming);
};
}  // namespace at
}  // namespace gui
}  // namespace tobas
