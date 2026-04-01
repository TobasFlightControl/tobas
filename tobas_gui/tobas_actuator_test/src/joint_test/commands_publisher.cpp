// SPDX-License-Identifier: GPL-3.0-or-later
// Copyright (C) 2026 Tobas, Inc.

#include "tobas_actuator_test/joint_test/commands_publisher.hpp"

#include <tobas_constants/ros_interface.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_qt_tools/util.hpp>
#include <tobas_std_tools/array.hpp>
#include <tobas_std_tools/check.hpp>

namespace tobas
{
namespace gui
{
namespace at
{
JointCommandsPublisherWidget::JointCommandsPublisherWidget(
  rclcpp::Node::SharedPtr node,
  const kdl::Tree& tree,
  const Drone& drone)
  : node_(node), tree_(tree), drone_(drone), joint_parser_(tree)
{
  rows_ = new QVBoxLayout();
  setLayout(rows_);

  connect(&publish_timer_, &QTimer::timeout, this, &self::publishCurrentValues);
}

void JointCommandsPublisherWidget::updateInternalDataStructures()
{
  TOBAS_CHECK(joint_parser_.updateInternalDataStructures());

  // 初期化
  commanders_.clear();
  qt::clearLayout(rows_);

  // アクティブ回転ジョイントのコマンダーを作成
  for (const auto& [jnt_name, joint] : drone_.joints) {
    if (!joint.isServoJoint()) {
      continue;
    }
    if (joint_parser_.joint(jnt_name).type != kdl::Joint::kRotation) {
      continue;
    }

    const auto commander = new qt::DoubleSliderDisplay();
    commander->setDecimals(3);
    commander->setText(QString::fromStdString(jnt_name));
    commander->setEnabled(false);

    switch (joint.cmd_iface) {
      case JointCommandInterface::kPosition: {
        const auto min_pos = joint_parser_.lowerLimit(jnt_name);
        const auto max_pos = joint_parser_.upperLimit(jnt_name);
        if (std::isinf(min_pos) || std::isinf(max_pos)) {
          qt::qErrorBox(this, "The position limit of joint \"" + QString::fromStdString(jnt_name) + "\" is invalid.");
          continue;
        }

        commander->setMinimum(min_pos);
        commander->setMaximum(max_pos);
        commander->setValue(0., true);
        commander->setSuffix(" rad");

        break;
      }
      case JointCommandInterface::kVelocity: {
        auto max_vel = joint_parser_.maxVelocity(jnt_name);
        if (std::isinf(max_vel)) {
          max_vel = kDefaultMaxVel;
        }

        commander->setMinimum(-max_vel);
        commander->setMaximum(max_vel);
        commander->setValue(0., true);
        commander->setSuffix(" rad/s");

        break;
      }
      case JointCommandInterface::kEffort: {
        auto max_eff = joint_parser_.maxEffort(jnt_name);
        if (std::isinf(max_eff)) {
          max_eff = kDefaultMaxEff;
        }

        commander->setMinimum(-max_eff);
        commander->setMaximum(max_eff);
        commander->setValue(0., true);
        commander->setSuffix(" Nm");

        break;
      }
      case JointCommandInterface::kNone: {
        qt::qErrorBox(this, "The command interface of joint \"" + QString::fromStdString(jnt_name) + "\" is not set.");
        continue;
      }
      default: {
        throw;
      }
    }

    rows_->addWidget(commander);
    connect(commander, &qt::DoubleSliderDisplay::valueChanged, this, &self::onValueChanged);
    commanders_[jnt_name] = commander;
  }

  // トピックを更新
  const auto ns = '/' + drone_.name;
  const auto pos_topic = path::join(ns, kRemoteIfaceNS, topic::kJointPosCmd);
  const auto vel_topic = path::join(ns, kRemoteIfaceNS, topic::kJointVelCmd);
  const auto eff_topic = path::join(ns, kRemoteIfaceNS, topic::kJointEffCmd);
  pos_pub_ = ros2::createPublisher<tobas_msgs::msg::JointCommandArray>(node_, pos_topic);
  vel_pub_ = ros2::createPublisher<tobas_msgs::msg::JointCommandArray>(node_, vel_topic);
  eff_pub_ = ros2::createPublisher<tobas_msgs::msg::JointCommandArray>(node_, eff_topic);
}

void JointCommandsPublisherWidget::start()
{
  // コマンダーを有効化
  for (const auto& [jnt_name, commander] : commanders_) {
    const auto& joint = drone_.joints.at(jnt_name);
    switch (joint.cmd_iface) {
      case JointCommandInterface::kPosition:
        commander->setValue(joint.home_pos, true);
        break;
      case JointCommandInterface::kVelocity:
        commander->setValue(0., true);
        break;
      case JointCommandInterface::kEffort:
        commander->setValue(0., true);
        break;
      case JointCommandInterface::kNone:
      default:
        throw;
    }

    commander->setEnabled(true);
  }

  // ジョイントがリセットされないよう一定周期でコマンドを発行し続ける
  publish_timer_.start(kPublishPeriod);
}

void JointCommandsPublisherWidget::stop()
{
  // コマンダーを無効化
  for (const auto& [jnt_name, commander] : commanders_) {
    commander->setValue(0., true);
    commander->setEnabled(false);
  }

  // タイマーを停止
  publish_timer_.stop();
}

void JointCommandsPublisherWidget::setZero()
{
  for (const auto& [_, commander] : commanders_) {
    commander->setValue(0.);
  }
}

void JointCommandsPublisherWidget::setHome()
{
  for (const auto& [jnt_name, commander] : commanders_) {
    const auto& joint = drone_.joints.at(jnt_name);
    switch (joint.cmd_iface) {
      case JointCommandInterface::kPosition:
        commander->setValue(joint.home_pos);
        break;
      case JointCommandInterface::kVelocity:
        commander->setValue(0.);
        break;
      case JointCommandInterface::kEffort:
        commander->setValue(0.);
        break;
      case JointCommandInterface::kNone:
      default:
        throw;
    }
  }
}

size_t JointCommandsPublisherWidget::numRegisteredChannels() const
{
  return commanders_.size();
}

void JointCommandsPublisherWidget::publishCurrentValues()
{
  // Create messages
  auto tar_pos = std::make_unique<tobas_msgs::msg::JointCommandArray>();
  auto tar_vel = std::make_unique<tobas_msgs::msg::JointCommandArray>();
  auto tar_eff = std::make_unique<tobas_msgs::msg::JointCommandArray>();

  // Fill messages
  for (const auto& [jnt_name, commander] : commanders_) {
    const auto& joint = drone_.joints.at(jnt_name);
    switch (joint.cmd_iface) {
      case JointCommandInterface::kPosition:
        tar_pos->commands.emplace_back();
        tar_pos->commands.back().name = jnt_name;
        tar_pos->commands.back().data = commander->getValue();
        break;
      case JointCommandInterface::kVelocity:
        tar_vel->commands.emplace_back();
        tar_vel->commands.back().name = jnt_name;
        tar_vel->commands.back().data = commander->getValue();
        break;
      case JointCommandInterface::kEffort:
        tar_eff->commands.emplace_back();
        tar_eff->commands.back().name = jnt_name;
        tar_eff->commands.back().data = commander->getValue();
        break;
      case JointCommandInterface::kNone:
      default:
        throw;
    }
  }

  // Publish messages
  if (!tar_pos->commands.empty()) {
    tar_pos->header.stamp = node_->now();
    pos_pub_->publish(std::move(tar_pos));
  }
  if (!tar_vel->commands.empty()) {
    tar_vel->header.stamp = node_->now();
    vel_pub_->publish(std::move(tar_vel));
  }
  if (!tar_eff->commands.empty()) {
    tar_eff->header.stamp = node_->now();
    eff_pub_->publish(std::move(tar_eff));
  }
}

void JointCommandsPublisherWidget::publishTimerCb()
{
  publishCurrentValues();
}

void JointCommandsPublisherWidget::onValueChanged()
{
  publishCurrentValues();
}
}  // namespace at
}  // namespace gui
}  // namespace tobas
