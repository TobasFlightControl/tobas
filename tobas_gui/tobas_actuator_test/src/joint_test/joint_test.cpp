// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_actuator_test/joint_test/joint_test.hpp"

#include <tobas_gui_common/constants.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/widgets/description_widget.hpp>

namespace tobas
{
namespace gui
{
namespace at
{
JointTestWidget::JointTestWidget(const rqt::RosQtBridge& bridge, const kdl::Tree& tree, const Drone& drone)
  : tree_(tree), drone_(drone)
{
  constexpr int kButtonWidth = 100;
  constexpr int kButtonHeight = 40;

  const auto instruction = new qt::DescriptionWidget(
    "1. Click 'Start' to start joint test.\n\n"
    "2. For each channel, confirm that the position, velocity, or effort is correctly following the command.\n\n"
    "3. If any joint does not behave as expected, please review the UADF or Setup Assistant settings.\n\n"
    "4. Click 'Stop' to stop joint test.\n\n",
    cmn::kBodyPSize);

  start_stop_button_ = new qt::ToggleButton("Start", "Stop");
  start_stop_button_->setFixedSize(kButtonWidth, kButtonHeight);

  zero_button_ = new QPushButton("Zero");
  zero_button_->setFixedSize(kButtonWidth, kButtonHeight);

  home_button_ = new QPushButton("Home");
  home_button_->setFixedSize(kButtonWidth, kButtonHeight);

  commands_publisher_ = new JointCommandsPublisherWidget(tree, drone);

  // Layout
  const auto cols = new QHBoxLayout();
  cols->addWidget(start_stop_button_);
  cols->addStretch();
  cols->addWidget(zero_button_);
  cols->addWidget(home_button_);

  rows_->addWidget(instruction);
  rows_->addLayout(cols);
  rows_->addWidget(commands_publisher_);
  rows_->addStretch();

  // Connection
  connect(start_stop_button_, &qt::ToggleButton::checked, this, &self::onStartButtonClicked);
  connect(start_stop_button_, &qt::ToggleButton::unchecked, this, &self::onStopButtonClicked);
  connect(zero_button_, &QPushButton::clicked, this, &self::onZeroButtonClicked);
  connect(home_button_, &QPushButton::clicked, this, &self::onHomeButtonClicked);
  connect(&bridge, &rqt::RosQtBridge::armingReceived, this, &self::armingCb, Qt::QueuedConnection);

  updateActionAvailability();
}

const char* JointTestWidget::title() const
{
  return "Test Servo Joints";
}

void JointTestWidget::reset()
{
  commands_publisher_->stop();

  running_ = false;
  arming_.reset();

  updateActionAvailability();
}

void JointTestWidget::updateInternalDataStructures()
{
  commands_publisher_->updateInternalDataStructures();
}

void JointTestWidget::initializeRosInterfaces(rclcpp::Node::SharedPtr node, const std::string& ns)
{
  commands_publisher_->initializeRosInterfaces(std::move(node), ns);
  ros_initialized_ = true;
}

void JointTestWidget::clearRosInterfaces()
{
  commands_publisher_->clearRosInterfaces();
  ros_initialized_ = false;
}

int JointTestWidget::numRegisteredChannels() const
{
  return commands_publisher_->numRegisteredChannels();
}

void JointTestWidget::updateActionAvailability()
{
  const auto disarmed = arming_ && !arming_->data;

  start_stop_button_->setChecked(running_);
  start_stop_button_->setEnabled(running_ || (ros_initialized_ && disarmed));

  const auto ctrl_buttons_enabled = ros_initialized_ && running_ && disarmed;
  zero_button_->setEnabled(ctrl_buttons_enabled);
  home_button_->setEnabled(ctrl_buttons_enabled);
}

void JointTestWidget::onStartButtonClicked()
{
  commands_publisher_->start();

  running_ = true;
  updateActionAvailability();

  qt::qInfoBox(this, "Joint test started.");
}

void JointTestWidget::onStopButtonClicked()
{
  commands_publisher_->setHome();

  reset();

  qt::qInfoBox(this, "Joint test stopped.");
}

void JointTestWidget::onZeroButtonClicked()
{
  commands_publisher_->setZero();
}

void JointTestWidget::onHomeButtonClicked()
{
  commands_publisher_->setHome();
}

void JointTestWidget::armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming)
{
  arming_ = arming;

  updateActionAvailability();
}
}  // namespace at
}  // namespace gui
}  // namespace tobas
