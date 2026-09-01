// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_simulation_gui/commanders/joint_commander.hpp"

#include <QDebug>
#include <QHBoxLayout>

#include <tobas_constants/ros_interface.hpp>
#include <tobas_gui_common/constants.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/util.hpp>
#include <tobas_qt_tools/widgets/label.hpp>

#include "tobas_simulation_gui/commanders/constants.hpp"

namespace ch = std::chrono;

namespace tobas
{
namespace gui
{
namespace sim
{
namespace
{
constexpr int kPublishCommandPeriod = 100;  // [ms]
}  // namespace

JointCommanderWidget::JointCommanderWidget(const kdl::Tree& tree, const Drone& drone)
  : tree_(tree), drone_(drone), rnd_gen_(rnd_dev_()), joint_parser_(tree)
{
  const auto title = new qt::Label("User Joint", cmn::kLabelPSize, QFont::Bold);

  start_stop_button_ = new qt::ToggleButton("Start", "Stop");
  start_stop_button_->setFixedSize(kHeaderButtonWidth, kHeaderButtonHeight);

  cmd_rows_ = new QVBoxLayout();

  home_button_ = new QPushButton("Home");
  center_button_ = new QPushButton("Center");
  random_button_ = new QPushButton("Random");
  home_button_->setFixedHeight(kCommandButtonHeight);
  center_button_->setFixedHeight(kCommandButtonHeight);
  random_button_->setFixedHeight(kCommandButtonHeight);

  // Layout
  const auto header_cols = new QHBoxLayout();
  header_cols->addWidget(title);
  header_cols->addStretch();
  header_cols->addWidget(start_stop_button_);

  const auto button_cols = new QHBoxLayout();
  button_cols->addWidget(home_button_);
  button_cols->addWidget(center_button_);
  button_cols->addWidget(random_button_);

  const auto root_rows = new QVBoxLayout();
  root_rows->addLayout(header_cols);
  root_rows->addLayout(cmd_rows_);
  root_rows->addLayout(button_cols);

  setLayout(root_rows);

  // Connection
  connect(start_stop_button_, &qt::ToggleButton::checked, this, &self::onStartRequested);
  connect(start_stop_button_, &qt::ToggleButton::unchecked, this, &self::onStopRequested);
  connect(home_button_, &QPushButton::clicked, this, &self::onHomeButtonClicked);
  connect(center_button_, &QPushButton::clicked, this, &self::onCenterButtonClicked);
  connect(random_button_, &QPushButton::clicked, this, &self::onRandomButtonClicked);
  connect(&publish_cmd_timer_, &QTimer::timeout, this, &self::onPublishCommandTimerTimeout);

  reset();
}

void JointCommanderWidget::reset()
{
  start_stop_button_->setChecked(false);

  for (const auto& [_, commander] : commanders_) {
    commander->setValue(0.0);
    commander->setEnabled(false);
  }

  for (auto& cmd : tar_js_pos_.commands) {
    cmd.data = 0.0;
  }
  for (auto& cmd : tar_js_vel_.commands) {
    cmd.data = 0.0;
  }
  for (auto& cmd : tar_js_eff_.commands) {
    cmd.data = 0.0;
  }

  home_button_->setEnabled(false);
  center_button_->setEnabled(false);
  random_button_->setEnabled(false);

  publish_cmd_timer_.stop();
}

void JointCommanderWidget::updateInternalDataStructures()
{
  if (!joint_parser_.updateInternalDataStructures()) {
    qt::qErrorBox(this, "Failed to update joint parser.");
    return;
  }

  // Clear joints for previous robot.
  commanders_.clear();
  qt::clearLayout(cmd_rows_);

  tar_js_pos_.commands.clear();
  tar_js_vel_.commands.clear();
  tar_js_eff_.commands.clear();

  // Add the current robot's joints.
  for (const auto& [jnt_name, joint] : drone_.joints) {
    // Include only user-controlled joints.
    if (joint.role != JointRole::kUserActive) {
      continue;
    }

    // Check that the joint is not fixed.
    const auto& jnt_type = joint_parser_.joint(jnt_name).type;
    if (jnt_type == kdl::Joint::kFixed) {
      qWarning() << "The joint type of" << QString::fromStdString(jnt_name) << "is FIXED.";
      continue;
    }

    const auto commander = new qt::DoubleSliderDisplay();
    commander->setText(QString::fromStdString(jnt_name));

    switch (joint.cmd_iface) {
      case JointCommandInterface::kPosition: {
        commander->setMinimum(joint_parser_.lowerLimit(jnt_name));
        commander->setMaximum(joint_parser_.upperLimit(jnt_name));
        switch (jnt_type) {
          case kdl::Joint::kRotation:
            commander->setSuffix(" rad");
            break;
          case kdl::Joint::kTranslation:
            commander->setSuffix(" m");
            break;
          case kdl::Joint::kFixed:
          default:
            throw std::runtime_error("Unexpected joint command interface type: " + std::to_string((int)joint.cmd_iface));
        }

        tobas_msgs::msg::JointCommand cmd;
        cmd.name = jnt_name;
        cmd.data = joint.home_pos;
        tar_js_pos_.commands.push_back(cmd);

        break;
      }
      case JointCommandInterface::kVelocity: {
        const auto max_vel = joint_parser_.maxVelocity(jnt_name);
        commander->setMinimum(-max_vel);
        commander->setMaximum(+max_vel);
        switch (jnt_type) {
          case kdl::Joint::kRotation:
            commander->setSuffix(" rad/s");
            break;
          case kdl::Joint::kTranslation:
            commander->setSuffix(" m/s");
            break;
          case kdl::Joint::kFixed:
          default:
            throw std::runtime_error("Unexpected joint command interface type: " + std::to_string((int)joint.cmd_iface));
        }

        tobas_msgs::msg::JointCommand cmd;
        cmd.name = jnt_name;
        cmd.data = 0.0;
        tar_js_vel_.commands.push_back(cmd);

        break;
      }
      case JointCommandInterface::kEffort: {
        const auto max_eff = joint_parser_.maxEffort(jnt_name);
        commander->setMinimum(-max_eff);
        commander->setMaximum(+max_eff);
        switch (jnt_type) {
          case kdl::Joint::kRotation:
            commander->setSuffix(" Nm");
            break;
          case kdl::Joint::kTranslation:
            commander->setSuffix(" N");
            break;
          case kdl::Joint::kFixed:
          default:
            throw std::runtime_error("Unexpected joint command interface type: " + std::to_string((int)joint.cmd_iface));
        }

        tobas_msgs::msg::JointCommand cmd;
        cmd.name = jnt_name;
        cmd.data = 0.0;
        tar_js_eff_.commands.push_back(cmd);

        break;
      }
      case JointCommandInterface::kNone: {
        break;
      }
      default: {
        qt::qErrorBox(this, "The command interface of joint " + QString::fromStdString(jnt_name) + " is invalid.");
        continue;
      }
    }

    commander->setValue(0.0);
    commander->setEnabled(false);
    connect(
      commander,
      &qt::DoubleSliderDisplay::valueChanged,
      std::bind(&self::onValueChanged, this, std::placeholders::_1, jnt_name));
    commanders_[jnt_name] = commander;
    cmd_rows_->addWidget(commander);
  }

  // Enable joint commander only if at least one commander exists.
  start_stop_button_->setEnabled(!commanders_.empty());
}

void JointCommanderWidget::initializeRosInterfaces(rclcpp::Node::SharedPtr node, const std::string& ns)
{
  if (!tar_js_pos_.commands.empty()) {
    tar_js_pos_pub_ = ros2::createPublisher<CmdMsg>(node, path::join(ns, topic::kJointPosCmd));
  }
  if (!tar_js_vel_.commands.empty()) {
    tar_js_vel_pub_ = ros2::createPublisher<CmdMsg>(node, path::join(ns, topic::kJointVelCmd));
  }
  if (!tar_js_eff_.commands.empty()) {
    tar_js_eff_pub_ = ros2::createPublisher<CmdMsg>(node, path::join(ns, topic::kJointEffCmd));
  }
}

void JointCommanderWidget::clearRosInterfaces()
{
  tar_js_pos_pub_.reset();
  tar_js_vel_pub_.reset();
  tar_js_eff_pub_.reset();
}

void JointCommanderWidget::publishCurrentCommand()
{
  if (tar_js_pos_pub_) {
    auto tar_js_pos = std::make_unique<CmdMsg>(tar_js_pos_);
    tar_js_pos_pub_->publish(std::move(tar_js_pos));
  }

  if (tar_js_vel_pub_) {
    auto tar_js_vel = std::make_unique<CmdMsg>(tar_js_vel_);
    tar_js_vel_pub_->publish(std::move(tar_js_vel));
  }

  if (tar_js_eff_pub_) {
    auto tar_js_eff = std::make_unique<CmdMsg>(tar_js_eff_);
    tar_js_eff_pub_->publish(std::move(tar_js_eff));
  }
}

void JointCommanderWidget::onStartRequested()
{
  // Set the initial command to the home position and enable it.
  for (const auto& [jnt_name, commander] : commanders_) {
    commander->setValue(drone_.joints.at(jnt_name).home_pos);
    commander->setEnabled(true);
  }

  // Enable command buttons.
  home_button_->setEnabled(true);
  center_button_->setEnabled(true);
  random_button_->setEnabled(true);

  // Start sending commands at fixed time intervals.
  publish_cmd_timer_.start(kPublishCommandPeriod);

  qt::qInfoBox(this, "GUI teleoperation is ready.");
}

void JointCommanderWidget::onStopRequested()
{
  reset();

  qt::qInfoBox(this, "GUI teleoperation is ready.");
}

void JointCommanderWidget::onValueChanged(double value, const std::string& jnt_name)
{
  if (!drone_.joints.contains(jnt_name)) {
    qWarning() << "Invalid joint name:" << QString::fromStdString(jnt_name);
    return;
  }

  const auto& joint = drone_.joints.at(jnt_name);
  bool jnt_found = false;

  switch (joint.cmd_iface) {
    case JointCommandInterface::kPosition: {
      for (auto& cmd : tar_js_pos_.commands) {
        if (cmd.name == jnt_name) {
          cmd.data = value;
          jnt_found = true;
          break;
        }
      }

      if (!jnt_found) {
        qWarning() << "Position commanded joint" << QString::fromStdString(jnt_name) << "is not found.";
        return;
      }

      break;
    }
    case JointCommandInterface::kVelocity: {
      for (auto& cmd : tar_js_vel_.commands) {
        if (cmd.name == jnt_name) {
          cmd.data = value;
          jnt_found = true;
          break;
        }
      }

      if (!jnt_found) {
        qWarning() << "Velocity commanded joint" << QString::fromStdString(jnt_name) << "is not found.";
        return;
      }

      break;
    }
    case JointCommandInterface::kEffort: {
      for (auto& cmd : tar_js_eff_.commands) {
        if (cmd.name == jnt_name) {
          cmd.data = value;
          jnt_found = true;
          break;
        }
      }

      if (!jnt_found) {
        qWarning() << "Effort commanded joint" << QString::fromStdString(jnt_name) << "is not found.";
        return;
      }

      break;
    }
    case JointCommandInterface::kNone: {
      break;
    }
    default: {
      qWarning() << "The command interface of joint" << QString::fromStdString(jnt_name) << "is invalid.";
      return;
    }
  }

  publishCurrentCommand();
}

void JointCommanderWidget::onHomeButtonClicked()
{
  for (const auto& [jnt_name, commander] : commanders_) {
    commander->setValue(drone_.joints.at(jnt_name).home_pos);
  }

  publishCurrentCommand();
}

void JointCommanderWidget::onCenterButtonClicked()
{
  for (const auto& [_, commander] : commanders_) {
    commander->setValue((commander->getMinimum() + commander->getMaximum()) / 2);
  }

  publishCurrentCommand();
}

void JointCommanderWidget::onRandomButtonClicked()
{
  for (const auto& [_, commander] : commanders_) {
    std::uniform_real_distribution<double> uniform(commander->getMinimum(), commander->getMaximum());
    const auto value = uniform(rnd_gen_);
    commander->setValue(value);
  }
}

void JointCommanderWidget::onPublishCommandTimerTimeout()
{
  publishCurrentCommand();
}
}  // namespace sim
}  // namespace gui
}  // namespace tobas
