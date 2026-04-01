// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <random>

#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>

#include <tobas_drone_core/drone.hpp>
#include <tobas_kdl/tree_joint_parser.hpp>
#include <tobas_qt_tools/widgets/slider_display.hpp>
#include <tobas_qt_tools/widgets/toggle_button.hpp>
#include <tobas_ros2_tools/register.hpp>

#include <tobas_msgs/msg/joint_command_array.hpp>

namespace tobas
{
namespace gui
{
namespace sim
{
class JointCommanderWidget : public QWidget
{
  Q_OBJECT

  using self = JointCommanderWidget;
  using super = QWidget;
  using CmdMsg = tobas_msgs::msg::JointCommandArray;

  static constexpr int kPublishCommandPeriod = 100;  // [ms]

public:
  explicit JointCommanderWidget(rclcpp::Node::SharedPtr node, const kdl::Tree& tree, const Drone& drone);

  void updateInternalDataStructures();

  bool start();
  void reset();

private:
  const rclcpp::Node::SharedPtr node_;
  const kdl::Tree& tree_;
  const Drone& drone_;

  std::random_device rnd_dev_;
  std::mt19937 rnd_gen_;

  kdl::TreeJointParser joint_parser_;

  qt::ToggleButton* start_stop_button_;

  std::map<std::string, qt::DoubleSliderDisplay*> commanders_;
  QVBoxLayout* cmd_rows_;

  QPushButton* home_button_;
  QPushButton* center_button_;
  QPushButton* random_button_;

  CmdMsg tar_js_pos_;
  CmdMsg tar_js_vel_;
  CmdMsg tar_js_eff_;

  ros2::PublisherPtr<CmdMsg> tar_js_pos_pub_;
  ros2::PublisherPtr<CmdMsg> tar_js_vel_pub_;
  ros2::PublisherPtr<CmdMsg> tar_js_eff_pub_;

  QTimer publish_cmd_timer_;

  void publishCurrentCommand();

private Q_SLOTS:
  void onStartRequested();
  void onStopRequested();

  void onValueChanged(double value, const std::string& jnt_name);

  void onHomeButtonClicked();
  void onCenterButtonClicked();
  void onRandomButtonClicked();

  void onPublishCommandTimerTimeout();
};
}  // namespace sim
}  // namespace gui
}  // namespace tobas
