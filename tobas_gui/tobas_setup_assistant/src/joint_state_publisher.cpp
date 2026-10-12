// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/joint_state_publisher.hpp"

#include <algorithm>
#include <cmath>
#include <functional>

#include <QDebug>
#include <QHBoxLayout>

#include <tobas_math/definitions.hpp>
#include <tobas_qt_tools/util.hpp>
#include <tobas_qt_tools/widgets/scroll_area.hpp>
#include <tobas_ros2_tools/register.hpp>
#include <tobas_std_tools/vector.hpp>

namespace tobas
{
namespace gui
{
namespace sa
{
JointStatePublisherWidget::JointStatePublisherWidget(
  rclcpp::Node::SharedPtr node,
  const uadf::Model& uadf,
  const kdl::Tree& tree)
  : node_(node), uadf_(uadf), tree_(tree), rnd_gen_(rnd_dev_())
{
  slider_rows_ = new QVBoxLayout();

  const auto scroll_area = new qt::ScrollArea();
  scroll_area->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
  scroll_area->setLayout(slider_rows_);

  zero_button_ = new QPushButton("Zero");
  center_button_ = new QPushButton("Center");
  random_button_ = new QPushButton("Random");

  constexpr int kButtonHeight = 40;
  zero_button_->setFixedHeight(kButtonHeight);
  center_button_->setFixedHeight(kButtonHeight);
  random_button_->setFixedHeight(kButtonHeight);

  setControlButtonsEnabled(false);

  // Layout
  const auto button_cols = new QHBoxLayout();
  button_cols->addWidget(zero_button_);
  button_cols->addWidget(center_button_);
  button_cols->addWidget(random_button_);

  const auto rows = new QVBoxLayout();
  rows->addWidget(scroll_area);
  rows->addLayout(button_cols);

  setLayout(rows);

  // Connection
  connect(zero_button_, &QPushButton::clicked, this, &self::onZeroButtonClicked);
  connect(center_button_, &QPushButton::clicked, this, &self::onCenterButtonClicked);
  connect(random_button_, &QPushButton::clicked, this, &self::onRandomButtonClicked);
  connect(&publish_timer_, &QTimer::timeout, this, &self::publishCurrentPositions);

  // Register publishers.
  js_pub_ = ros2::createPublisher<sensor_msgs::msg::JointState>(node_, "joint_states");
  drs_pub_ =
    ros2::createPublisher<tobas_visualization_msgs::msg::DisplayRobotState>(node_, "display_robot_state", false, true);
}

void JointStatePublisherWidget::updateInternalDataStructures()
{
  js_.name.clear();
  js_.position.clear();
  sliders_.clear();
  thrust_joints_.clear();
  qt::clearLayout(slider_rows_);

  for (const auto& [_, elem] : tree_.getSegments()) {
    const auto& joint = elem.segment.joint();
    if (joint.type == kdl::Joint::kFixed) {
      continue;
    }

    js_.name.push_back(joint.name);
    js_.position.push_back(0.0);

    const auto thrust_it = uadf_.thrusts.find(joint.name);
    if (thrust_it != uadf_.thrusts.end()) {  // Rotate `thrust` joints automatically.
      const auto rotation_sign = thrust_it->second.direction == uadf::Thrust::CCW ? 1.0 : -1.0;
      thrust_joints_.push_back({ js_.position.size() - 1, rotation_sign });

      qDebug() << QString::fromStdString(joint.name) << "will rotate automatically.";
    }
    else {  // Allow users to control other movable joints with sliders.
      const auto slider = new qt::IntSliderDisplay();
      slider->setText(QString::fromStdString(joint.name));

      const auto is_rotation = joint.type == kdl::Joint::kRotation;
      const auto position_per_step = is_rotation ? math::kDeg2Rad : 0.001;

      auto lower_limit = joint.lower_limit;
      auto upper_limit = joint.upper_limit;
      if (is_rotation && upper_limit - lower_limit > M_2PI) {
        lower_limit = -M_PI;
        upper_limit = +M_PI;
      }
      slider->setRange(std::round(lower_limit / position_per_step), std::round(upper_limit / position_per_step));
      slider->setValue(0);
      slider->setSuffix(is_rotation ? " deg" : " mm");

      connect(
        slider,
        &qt::IntSliderDisplay::valueChanged,
        this,
        std::bind(&self::onValueChanged, this, std::placeholders::_1, joint.name, position_per_step));

      sliders_.push_back(slider);
      slider_rows_->addWidget(slider);

      qDebug() << QString::fromStdString(joint.name) << "was added to the JSP slider.";
    }
  }

  slider_rows_->addStretch();

  // Enable the control buttons only when controllable joints exist.
  setControlButtonsEnabled(!sliders_.empty());

  // Start to publish joint states.
  constexpr int kPublishPositionsInterval = 10;  // [ms]
  publish_timer_.start(kPublishPositionsInterval);
  thrust_rotation_timer_.start();
}

void JointStatePublisherWidget::publishCurrentPositions()
{
  // Update the thrust joint positions.
  const auto elapsed_sec = static_cast<double>(thrust_rotation_timer_.restart()) / 1000.0;
  constexpr double kThrustJointAngularVelocity = M_PI / 6;  // [rad/s]
  for (const auto& thrust_joint : thrust_joints_) {
    auto& position = js_.position.at(thrust_joint.state_index);
    position = std::remainder(position + thrust_joint.rotation_sign * kThrustJointAngularVelocity * elapsed_sec, M_2PI);
  }

  js_.header.stamp = node_->now();

  auto js = make_unique<sensor_msgs::msg::JointState>(js_);
  js_pub_->publish(std::move(js));

  auto drs = std::make_unique<tobas_visualization_msgs::msg::DisplayRobotState>();
  drs->state.joint_state = js_;
  drs->state.is_diff = js_.name.empty();  // Mark empty states as diffs to avoid RViz's empty JointState error.
  drs_pub_->publish(std::move(drs));
}

void JointStatePublisherWidget::setControlButtonsEnabled(bool enabled)
{
  zero_button_->setEnabled(enabled);
  center_button_->setEnabled(enabled);
  random_button_->setEnabled(enabled);
}

void JointStatePublisherWidget::onValueChanged(int value, const std::string& jnt_name, double position_per_step)
{
  const auto idx = st::index(js_.name, jnt_name);
  if (idx < 0) {
    qWarning() << "Joint" << QString::fromStdString(jnt_name) << "does not exist.";
    return;
  }

  js_.position.at(idx) = value * position_per_step;
}

void JointStatePublisherWidget::onZeroButtonClicked()
{
  for (auto& slider : sliders_) {
    slider->setValue(0.0);
  }
}

void JointStatePublisherWidget::onCenterButtonClicked()
{
  for (auto& slider : sliders_) {
    const auto value = (slider->getMinimum() + slider->getMaximum()) / 2;
    slider->setValue(value);
  }
}

void JointStatePublisherWidget::onRandomButtonClicked()
{
  for (auto& slider : sliders_) {
    std::uniform_int_distribution<int> uniform(slider->getMinimum(), slider->getMaximum());
    const auto value = uniform(rnd_gen_);
    slider->setValue(value);
  }
}
}  // namespace sa
}  // namespace gui
}  // namespace tobas
