// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#pragma once

#include <random>

#include <QElapsedTimer>
#include <QPushButton>
#include <QTimer>
#include <QVBoxLayout>

#include <tobas_kdl/tree.hpp>
#include <tobas_math/definitions.hpp>
#include <tobas_qt_tools/widgets/slider_display.hpp>
#include <tobas_ros2_tools/definitions.hpp>
#include <tobas_uadf/model.hpp>

#include <sensor_msgs/msg/joint_state.hpp>

#include <tobas_visualization_msgs/msg/display_robot_state.hpp>

namespace rclcpp
{
class Node;
}  // namespace rclcpp

namespace tobas
{
namespace gui
{
namespace sa
{
class JointStatePublisherWidget : public QWidget
{
  Q_OBJECT

  using self = JointStatePublisherWidget;
  using super = QWidget;

public:
  explicit JointStatePublisherWidget(std::shared_ptr<rclcpp::Node> node, const uadf::Model& uadf, const kdl::Tree& tree);

  void updateInternalDataStructures();

private:
  struct ThrustJoint
  {
    size_t state_index;
    double rotation_sign;
  };

  const std::shared_ptr<rclcpp::Node> node_;
  const uadf::Model& uadf_;
  const kdl::Tree& tree_;

  std::random_device rnd_dev_;
  std::mt19937 rnd_gen_;

  QVBoxLayout* slider_rows_;
  QPushButton* zero_button_;
  QPushButton* center_button_;
  QPushButton* random_button_;

  sensor_msgs::msg::JointState js_;
  std::vector<qt::DoubleSliderDisplay*> sliders_;
  std::vector<ThrustJoint> thrust_joints_;

  ros2::PublisherPtr<sensor_msgs::msg::JointState> js_pub_;
  ros2::PublisherPtr<tobas_visualization_msgs::msg::DisplayRobotState> drs_pub_;

  QTimer publish_timer_;
  QElapsedTimer thrust_rotation_timer_;

  void publishCurrentPositions();
  void setControlButtonsEnabled(bool enabled);

private Q_SLOTS:
  void onValueChanged(double value, const std::string& jnt_name);

  void onZeroButtonClicked();
  void onCenterButtonClicked();
  void onRandomButtonClicked();
};
}  // namespace sa
}  // namespace gui
}  // namespace tobas
