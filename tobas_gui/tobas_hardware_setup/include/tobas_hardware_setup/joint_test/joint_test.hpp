#pragma once

#include <rclcpp/rclcpp.hpp>

#include <tobas_drone_core/drone.hpp>
#include <tobas_msgs/msg/arming.hpp>

#include "../base.hpp"
#include "./commands_publisher.hpp"

namespace gui
{
namespace hardware_setup
{
class JointTestWidget : public BaseHardwareSetupWidget
{
  Q_OBJECT

  using self = JointTestWidget;
  using super = BaseHardwareSetupWidget;

  static constexpr int kButtonWidth = 100;
  static constexpr int kButtonHeight = 40;

public:
  explicit JointTestWidget(rclcpp::Node::SharedPtr node, const kdl::Tree& tree, const tobas::Drone& drone);

  const char* name() const override;
  const char* title() const override;

  void updateInternalDataStructures();

private:
  const rclcpp::Node::SharedPtr node_;
  const kdl::Tree& tree_;
  const tobas::Drone& drone_;

  QPushButton* start_button_;
  QPushButton* stop_button_;
  JointCommandsPublisherWidget* commands_publisher_;

  tobas_msgs::msg::Arming::ConstSharedPtr arming_;

  ros2::SubscriberPtr<tobas_msgs::msg::Arming> arming_sub_;

  void reset();

  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming);

private Q_SLOTS:
  void onStartButtonClicked();
  void onStopButtonClicked();
};
}  // namespace hardware_setup
}  // namespace gui
