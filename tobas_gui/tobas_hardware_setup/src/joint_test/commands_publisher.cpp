#include "tobas_hardware_setup/joint_test/commands_publisher.hpp"

#include <QGridLayout>
#include <QVBoxLayout>

#include <tobas_constants/constants.hpp>
#include <tobas_path_tools/join.hpp>
#include <tobas_qt_tools/message.hpp>
#include <tobas_std_tools/array.hpp>
#include <tobas_std_tools/check.hpp>

namespace gui
{
namespace hw
{
JointCommandsPublisherWidget::JointCommandsPublisherWidget(
  rclcpp::Node::SharedPtr node,
  const kdl::Tree& tree,
  const tobas::Drone& drone)
  : node_(node), tree_(tree), drone_(drone), joint_parser_(tree)
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  const auto grid = new QGridLayout();
  rows->addLayout(grid);

  for (size_t ch = 0; ch < kChannelSize; ++ch) {
    commanders_[ch] = new qt::DoubleSliderDisplay();
    connect(commanders_[ch], &qt::DoubleSliderDisplay::valueChanged, this, &self::onValueChanged);
    grid->addWidget(commanders_[ch], ch % kMaxRows, ch / kMaxRows);
  }

  connect(&publish_timer_, &QTimer::timeout, this, &self::publishCurrentValues);
}

void JointCommandsPublisherWidget::updateInternalDataStructures()
{
  TOBAS_CHECK(joint_parser_.updateInternalDataStructures());

  // 初期化
  jnt_names_.fill("");
  cmd_iface_.fill(tobas::JointCommandInterface::kNone);
  home_pos_.fill(0.);
  registered_.fill(false);
  for (size_t ch = 0; ch < kChannelSize; ++ch) {
    commanders_[ch]->setText("CH" + QString::number(ch) + ": unregistered");
    commanders_[ch]->setMinimum(0.);
    commanders_[ch]->setMaximum(0.);
    commanders_[ch]->setSuffix("");
    commanders_[ch]->setEnabled(false);
  }

  // PWMサーボとして登録されているチャンネルの設定
  for (const auto& [_, joint] : drone_.joints) {
    if (joint.hw_iface != tobas::HardwareInterface::kPwm) {
      continue;
    }

    if (!drone_.pwms.contains(joint.name)) {
      qt::qErrorBox(this, "PWM configuration of joint \"" + QString::fromStdString(joint.name) + "\" does not exist.");
      continue;
    }

    const auto& pwm = drone_.pwms.at(joint.name);
    const auto& ch = pwm.channel;
    if (ch >= kChannelSize) {
      qt::qErrorBox(this, "Channel " + QString::number(ch) + " is out of range.");
      continue;
    }

    commanders_[ch]->setText("CH" + QString::number(ch) + ": " + QString::fromStdString(joint.name));

    switch (joint.cmd_iface) {
      case tobas::JointCommandInterface::kPosition: {
        const auto min_pos = joint_parser_.lowerLimit(joint.name);
        const auto max_pos = joint_parser_.upperLimit(joint.name);
        if (std::isinf(min_pos) || std::isinf(max_pos)) {
          qt::qErrorBox(this, "The position limit of joint \"" + QString::fromStdString(joint.name) + "\" is invalid.");
          continue;
        }

        commanders_[ch]->setMinimum(min_pos);
        commanders_[ch]->setMaximum(max_pos);
        commanders_[ch]->setValue(0., true);
        commanders_[ch]->setSuffix(" rad");

        break;
      }
      case tobas::JointCommandInterface::kVelocity: {
        auto max_vel = joint_parser_.maxVelocity(joint.name);
        if (std::isinf(max_vel)) {
          max_vel = kDefaultMaxVel;
        }

        commanders_[ch]->setMinimum(-max_vel);
        commanders_[ch]->setMaximum(max_vel);
        commanders_[ch]->setValue(0., true);
        commanders_[ch]->setSuffix(" rad/s");

        break;
      }
      case tobas::JointCommandInterface::kEffort: {
        auto max_eff = joint_parser_.maxEffort(joint.name);
        if (std::isinf(max_eff)) {
          max_eff = kDefaultMaxEff;
        }

        commanders_[ch]->setMinimum(-max_eff);
        commanders_[ch]->setMaximum(max_eff);
        commanders_[ch]->setValue(0., true);
        commanders_[ch]->setSuffix(" Nm");

        break;
      }
      case tobas::JointCommandInterface::kNone: {
        qt::qErrorBox(this, "The command interface of joint \"" + QString::fromStdString(joint.name) + "\" is not set.");
        continue;
      }
      default: {
        throw;
      }
    }

    jnt_names_[ch] = joint.name;
    cmd_iface_[ch] = joint.cmd_iface;
    home_pos_[ch] = joint.home_pos;
    registered_[ch] = true;
  }

  // トピックを更新
  const auto pos_topic = path::join(drone_.name, tobas::kRemoteIfaceTopicNS, tobas::kJointPosCmdTopic);
  const auto vel_topic = path::join(drone_.name, tobas::kRemoteIfaceTopicNS, tobas::kJointVelCmdTopic);
  const auto eff_topic = path::join(drone_.name, tobas::kRemoteIfaceTopicNS, tobas::kJointEffCmdTopic);
  pos_pub_ = ros2::createPublisher<tobas_msgs::msg::JointCommandArray>(node_, pos_topic);
  vel_pub_ = ros2::createPublisher<tobas_msgs::msg::JointCommandArray>(node_, vel_topic);
  eff_pub_ = ros2::createPublisher<tobas_msgs::msg::JointCommandArray>(node_, eff_topic);
}

void JointCommandsPublisherWidget::start()
{
  // コマンダーを有効化
  for (size_t ch = 0; ch < kChannelSize; ++ch) {
    switch (cmd_iface_[ch]) {
      case tobas::JointCommandInterface::kPosition:
        commanders_[ch]->setValue(home_pos_[ch], true);
        break;
      case tobas::JointCommandInterface::kVelocity:
        commanders_[ch]->setValue(0., true);
        break;
      case tobas::JointCommandInterface::kEffort:
        commanders_[ch]->setValue(0., true);
        break;
      case tobas::JointCommandInterface::kNone:
        continue;
      default:
        throw;
    }

    commanders_[ch]->setEnabled(true);
  }

  // ジョイントがリセットされないよう一定周期でコマンドを発行し続ける
  publish_timer_.start(kPublishPeriod);
}

void JointCommandsPublisherWidget::stop()
{
  // コマンダーを無効化
  for (size_t ch = 0; ch < kChannelSize; ++ch) {
    commanders_[ch]->setValue(0., true);
    commanders_[ch]->setEnabled(false);
  }

  // タイマーを停止
  publish_timer_.stop();
}

int JointCommandsPublisherWidget::numRegisteredChannels() const
{
  return tobas_std::count(registered_, true);
}

void JointCommandsPublisherWidget::publishCurrentValues()
{
  // Create messages
  auto tar_pos = std::make_unique<tobas_msgs::msg::JointCommandArray>();
  auto tar_vel = std::make_unique<tobas_msgs::msg::JointCommandArray>();
  auto tar_eff = std::make_unique<tobas_msgs::msg::JointCommandArray>();

  // Fill messages
  for (size_t ch = 0; ch < kChannelSize; ++ch) {
    switch (cmd_iface_[ch]) {
      case tobas::JointCommandInterface::kPosition:
        tar_pos->commands.emplace_back();
        tar_pos->commands.back().name = jnt_names_[ch];
        tar_pos->commands.back().data = commanders_[ch]->getValue();
        break;
      case tobas::JointCommandInterface::kVelocity:
        tar_vel->commands.emplace_back();
        tar_vel->commands.back().name = jnt_names_[ch];
        tar_vel->commands.back().data = commanders_[ch]->getValue();
        break;
      case tobas::JointCommandInterface::kEffort:
        tar_eff->commands.emplace_back();
        tar_eff->commands.back().name = jnt_names_[ch];
        tar_eff->commands.back().data = commanders_[ch]->getValue();
        break;
      case tobas::JointCommandInterface::kNone:
        continue;
      default:
        throw;
    }
  }

  // Publish messages
  if (tar_pos->commands.size() > 0) {
    tar_pos->header.stamp = node_->get_clock()->now();
    pos_pub_->publish(std::move(tar_pos));
  }
  if (tar_vel->commands.size() > 0) {
    tar_vel->header.stamp = node_->get_clock()->now();
    pos_pub_->publish(std::move(tar_vel));
  }
  if (tar_eff->commands.size() > 0) {
    tar_eff->header.stamp = node_->get_clock()->now();
    pos_pub_->publish(std::move(tar_eff));
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
}  // namespace hw
}  // namespace gui
