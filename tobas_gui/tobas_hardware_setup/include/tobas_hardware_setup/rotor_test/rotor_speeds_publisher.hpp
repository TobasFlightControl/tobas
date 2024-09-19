#pragma once

#include <QWidget>
#include <QPushButton>

#include <tobas_ros2_tools/register.hpp>
#include <tobas_drone_core/drone.hpp>
#include <tobas_qt_tools/widgets/slider_display.hpp>
#include <tobas_msgs/msg/rotor_speeds.hpp>

namespace gui
{
namespace hardware_setup
{
class RotorSpeedsPublisherWidget : public QWidget
{
  Q_OBJECT

  using self = RotorSpeedsPublisherWidget;
  using super = QWidget;

  static constexpr int kChannelSize = 14;  // TODO: ハードウェアの最大チャンネル数に合わせる
  static constexpr int kMaxRows = kChannelSize / 2;
  static constexpr int kButtonHeight = 50;
  static constexpr auto kPublishPeriod = std::chrono::milliseconds(100);
  static constexpr size_t kNumButtons = 6;
  static constexpr std::array<int, kNumButtons> kRPMs = { 0, 100, 500, 1000, 5000, 10000 };

public:
  explicit RotorSpeedsPublisherWidget(rclcpp::Node::SharedPtr node, const tobas::Drone& drone);

  void updateInternalDataStructures();

  void start();
  void stop();

private:
  const rclcpp::Node::SharedPtr node_;
  const tobas::Drone& drone_;

  std::array<qt::IntSliderDisplay*, kChannelSize> commanders_;
  std::array<QPushButton*, kNumButtons> rpm_buttons_;

  ros2::PublisherPtr<tobas_msgs::msg::RotorSpeeds> speeds_pub_;
  ros2::TimerPtr publish_timer_;

  void setAllValues(int value);
  void publishCurrentValues();

  void publishTimerCb();

private Q_SLOTS:
  void onValueChanged();
  void onRPMButtonClicked(int rpm);
};
}  // namespace hardware_setup
}  // namespace gui
