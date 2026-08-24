// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>
#include <QWidget>

#include <tobas_drone_core/drone.hpp>
#include <tobas_kdl/tree.hpp>
#include <tobas_kdl/tree_joint_parser.hpp>
#include <tobas_math/definitions.hpp>
#include <tobas_qt_tools/widgets/slider_display.hpp>
#include <tobas_ros2_tools/register.hpp>

#include <tobas_msgs/msg/joint_command_array.hpp>

namespace tobas
{
namespace gui
{
namespace at
{
class JointCommandsPublisherWidget : public QWidget
{
  Q_OBJECT

  using self = JointCommandsPublisherWidget;
  using super = QWidget;

  static constexpr int kPublishPeriod = 10;        // [ms]
  static constexpr double kDefaultMaxVel = M_2PI;  // [rad/s]
  static constexpr double kDefaultMaxEff = 10.0;   // [Nm]

public:
  explicit JointCommandsPublisherWidget(const kdl::Tree& tree, const Drone& drone);

  void updateInternalDataStructures();
  void initializeRosInterfaces(rclcpp::Node::SharedPtr node, const std::string& ns);

  void start();
  void stop();

  void setZero();
  void setHome();

  size_t numRegisteredChannels() const;

private:
  rclcpp::Node::SharedPtr node_;
  const kdl::Tree& tree_;
  const Drone& drone_;

  kdl::TreeJointParser joint_parser_;

  std::map<std::string, qt::DoubleSliderDisplay*> commanders_;
  QVBoxLayout* rows_;

  ros2::PublisherPtr<tobas_msgs::msg::JointCommandArray> pos_pub_;
  ros2::PublisherPtr<tobas_msgs::msg::JointCommandArray> vel_pub_;
  ros2::PublisherPtr<tobas_msgs::msg::JointCommandArray> eff_pub_;

  QTimer publish_timer_;

  void publishCurrentValues();

  void publishTimerCb();

private Q_SLOTS:
  void onValueChanged();
};
}  // namespace at
}  // namespace gui
}  // namespace tobas
