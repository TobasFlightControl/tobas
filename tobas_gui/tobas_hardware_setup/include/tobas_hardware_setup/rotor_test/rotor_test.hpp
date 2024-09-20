#pragma once

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>

#include <tobas_drone_core/drone.hpp>
#include <tobas_qt_tools/widgets/wait_spinner.hpp>

#include "../base.hpp"
#include "./rotor_speeds_publisher.hpp"
#include "./set_arm_thread.hpp"

namespace gui
{
namespace hardware_setup
{
class RotorTestWidget : public BaseHardwareSetupWidget
{
  Q_OBJECT

  using self = RotorTestWidget;
  using super = BaseHardwareSetupWidget;

  static constexpr int kButtonWidth = 100;
  static constexpr int kButtonHeight = 40;

public:
  explicit RotorTestWidget(rclcpp::Node::SharedPtr node, const tobas::Drone& drone);

  const char* name() const override;
  const char* title() const override;

  void onInit() override;

  void updateInternalDataStructures();

private:
  const rclcpp::Node::SharedPtr node_;
  const tobas::Drone& drone_;

  QPushButton* start_button_;
  QPushButton* stop_button_;
  RotorSpeedsPublisherWidget* speeds_publisher_;

  qt::WaitSpinnerWidget spinner_;
  SetArmThread arm_thread_;
  SetArmThread disarm_thread_;

  std_msgs::msg::Bool::ConstSharedPtr arming_;
  bool is_running_ = false;

  ros2::SubscriberPtr<std_msgs::msg::Bool> arming_sub_;

  void reset();

  void armingCb(const std_msgs::msg::Bool::ConstSharedPtr& arming);

private Q_SLOTS:
  void onStartButtonClicked();
  void onArmFinished(bool success, const QString& message);

  void onStopButtonClicked();
  void onDisarmFinished(bool success, const QString& message);
};
}  // namespace hardware_setup
}  // namespace gui
