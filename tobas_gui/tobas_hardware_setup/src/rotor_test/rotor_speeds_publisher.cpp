#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>

#include <tobas_std_tools/unit_conversions.hpp>
#include <tobas_path_tools/join.hpp>

#include "tobas_hardware_setup/rotor_test/rotor_speeds_publisher.hpp"

namespace gui
{
namespace hardware_setup
{
RotorSpeedsPublisherWidget::RotorSpeedsPublisherWidget(rclcpp::Node::SharedPtr node, const tobas::Drone& drone)
  : node_(node), drone_(drone)
{
  const auto rows = new QVBoxLayout();
  setLayout(rows);

  const auto grid = new QGridLayout();
  rows->addLayout(grid);

  for (size_t ch = 0; ch < kChannelSize; ++ch)
  {
    commanders_[ch] = new qt::IntSliderDisplay();
    commanders_[ch]->setSuffix(" rpm");
    connect(commanders_[ch], &qt::IntSliderDisplay::valueChanged, this, &self::onValueChanged);
    grid->addWidget(commanders_[ch], ch % kMaxRows, ch / kMaxRows);
  }

  const auto cols = new QHBoxLayout();
  rows->addLayout(cols);

  for (size_t i = 0; i < kNumButtons; ++i)
  {
    const auto& rpm = kRPMs[i];
    rpm_buttons_[i] = new QPushButton(QString::number(rpm) + " RPM");
    rpm_buttons_[i]->setFixedHeight(kButtonHeight);
    connect(rpm_buttons_[i], &QPushButton::clicked, bind(&self::onRPMButtonClicked, this, rpm));
    cols->addWidget(rpm_buttons_[i]);
  }
}

void RotorSpeedsPublisherWidget::updateInternalDataStructures()
{
  // モータとして登録されているチャンネルの設定
  std::unordered_set<size_t> rotor_channels;
  for (const auto& rotor : drone_.rotors)
  {
    const auto& ch = rotor.channel;
    rotor_channels.insert(ch);
    commanders_.at(ch)->setText("CH" + QString::number(ch) + ": " + QString::fromStdString(rotor.link_name));
    commanders_.at(ch)->setMaximum(tobas_std::rps2rpm(rotor.max_rot_speed));
    commanders_.at(ch)->setValue(0);
  }

  // モータとして登録されていないチャンネルを無効化
  for (size_t ch = 0; ch < kChannelSize; ++ch)
  {
    if (rotor_channels.contains(ch))
      continue;
    commanders_.at(ch)->setText("CH" + QString::number(ch) + ": unregistered");
    commanders_.at(ch)->setMaximum(0);
    commanders_.at(ch)->setEnabled(false);
  }

  // 回転数トピックを更新
  const auto speeds_topic = path::join(drone_.name, tobas::kRotorSpeedsCmdTopic);
  speeds_pub_ = ros2::createPublisher<tobas_msgs::msg::RotorSpeeds>(node_, speeds_topic);
}

void RotorSpeedsPublisherWidget::start()
{
  // コマンダーを有効化
  for (const auto& rotor : drone_.rotors)
  {
    commanders_.at(rotor.channel)->setValue(0);
    commanders_.at(rotor.channel)->setEnabled(true);
  }

  // RPMボタンを有効化
  for (const auto& button : rpm_buttons_)
    button->setEnabled(true);

  // モータが停止しないよう一定周期でコマンドを発行し続ける
  publish_timer_ = ros2::createTimer(node_, kPublishPeriod, &self::publishTimerCb, this);
}

void RotorSpeedsPublisherWidget::stop()
{
  // コマンダーを無効化
  for (const auto& rotor : drone_.rotors)
  {
    commanders_.at(rotor.channel)->setValue(0);
    commanders_.at(rotor.channel)->setEnabled(false);
  }

  // RPMボタンを無効化
  for (const auto& button : rpm_buttons_)
    button->setEnabled(false);

  // タイマーを停止
  publish_timer_->cancel();
}

void RotorSpeedsPublisherWidget::setAllValues(int value)
{
  for (const auto& rotor : drone_.rotors)
    commanders_.at(rotor.channel)->setValue(value);
}

void RotorSpeedsPublisherWidget::publishCurrentValues()
{
  auto rot_speeds = std::make_unique<tobas_msgs::msg::RotorSpeeds>();
  rot_speeds->header.stamp = node_->get_clock()->now();
  rot_speeds->speeds.resize(drone_.numRotors());

  for (const auto& rotor : drone_.rotors)
    rot_speeds->speeds.at(rotor.channel) = tobas_std::rpm2rps(commanders_.at(rotor.channel)->getValue());

  speeds_pub_->publish(std::move(rot_speeds));
}

void RotorSpeedsPublisherWidget::publishTimerCb()
{
  publishCurrentValues();
}

void RotorSpeedsPublisherWidget::onValueChanged()
{
  publishCurrentValues();
}

void RotorSpeedsPublisherWidget::onRPMButtonClicked(int rpm)
{
  setAllValues(rpm);
  publishCurrentValues();
}
}  // namespace hardware_setup
}  // namespace gui
