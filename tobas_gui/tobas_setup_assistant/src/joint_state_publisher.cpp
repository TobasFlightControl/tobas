// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_setup_assistant/joint_state_publisher.hpp"

#include <QHBoxLayout>
#include <QPushButton>

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
JointStatePublisherWidget::JointStatePublisherWidget(rclcpp::Node::SharedPtr node, const kdl::Tree& tree)
  : node_(node), tree_(tree), rnd_gen_(rnd_dev_())
{
  slider_rows_ = new QVBoxLayout();

  const auto scroll_area = new qt::ScrollArea();
  scroll_area->setSizePolicy(QSizePolicy::Preferred, QSizePolicy::Expanding);
  scroll_area->setLayout(slider_rows_);

  const auto zero_button = new QPushButton("Zero");
  const auto center_button = new QPushButton("Center");
  const auto random_button = new QPushButton("Random");

  zero_button->setFixedHeight(kButtonHeight);
  center_button->setFixedHeight(kButtonHeight);
  random_button->setFixedHeight(kButtonHeight);

  // Layout
  const auto button_cols = new QHBoxLayout();
  button_cols->addWidget(zero_button);
  button_cols->addWidget(center_button);
  button_cols->addWidget(random_button);

  const auto rows = new QVBoxLayout();
  rows->addWidget(scroll_area);
  rows->addLayout(button_cols);

  setLayout(rows);

  // Connection
  connect(zero_button, &QPushButton::clicked, this, &self::onZeroButtonClicked);
  connect(center_button, &QPushButton::clicked, this, &self::onCenterButtonClicked);
  connect(random_button, &QPushButton::clicked, this, &self::onRandomButtonClicked);
  connect(&publish_timer_, &QTimer::timeout, this, &self::publish);

  // Register publishers
  js_pub_ = ros2::createPublisher<sensor_msgs::msg::JointState>(node_, "joint_states");
  drs_pub_ =
    ros2::createPublisher<tobas_visualization_msgs::msg::DisplayRobotState>(node_, "display_robot_state", false, true);
}

void JointStatePublisherWidget::updateInternalDataStructures()
{
  js_.name.clear();
  js_.position.clear();
  sliders_.clear();
  qt::clearLayout(slider_rows_);

  for (const auto& [_, elem] : tree_.getSegments()) {
    const auto& joint = elem.segment.joint();
    if (joint.type == kdl::Joint::kFixed) {
      continue;
    }

    js_.name.push_back(joint.name);
    js_.position.push_back(0.);

    const auto slider = new qt::DoubleSliderDisplay();
    slider->setText(QString::fromStdString(joint.name));

    auto lower_limit = joint.lower_limit;
    auto upper_limit = joint.upper_limit;
    if (joint.type == kdl::Joint::kRotation && upper_limit - lower_limit > M_2PI) {
      lower_limit = -M_PI;
      upper_limit = +M_PI;
    }
    slider->setMinimum(lower_limit);
    slider->setMaximum(upper_limit);

    slider->setValue(0.);

    connect(
      slider,
      &qt::DoubleSliderDisplay::valueChanged,
      this,
      std::bind(&self::onValueChanged, this, std::placeholders::_1, joint.name));

    sliders_.push_back(slider);
    slider_rows_->addWidget(slider);

    RCLCPP_DEBUG_STREAM(node_->get_logger(), "\"" << joint.name << "\" is added to the JSP slider.");
  }

  slider_rows_->addStretch();

  // Start to publish joint states
  publish_timer_.start(100);
}

void JointStatePublisherWidget::publish()
{
  js_.header.stamp = node_->now();

  auto js = make_unique<sensor_msgs::msg::JointState>(js_);
  js_pub_->publish(std::move(js));

  auto drs = std::make_unique<tobas_visualization_msgs::msg::DisplayRobotState>();
  drs->state.joint_state = js_;
  drs_pub_->publish(std::move(drs));
}

void JointStatePublisherWidget::onValueChanged(double value, const std::string& jnt_name)
{
  const auto idx = st::index(js_.name, jnt_name);
  if (idx < 0) {
    RCLCPP_ERROR_STREAM(node_->get_logger(), "Joint \"" << jnt_name << "\" does not exist.");
    return;
  }

  js_.position.at(idx) = value;
}

void JointStatePublisherWidget::onZeroButtonClicked()
{
  for (auto& slider : sliders_) {
    slider->setValue(0.);
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
    std::uniform_real_distribution<double> uniform(slider->getMinimum(), slider->getMaximum());
    const auto value = uniform(rnd_gen_);
    slider->setValue(value);
  }
}
}  // namespace sa
}  // namespace gui
}  // namespace tobas
