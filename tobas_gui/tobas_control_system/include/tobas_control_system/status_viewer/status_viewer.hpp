#pragma once

#include <std_msgs/msg/bool.hpp>

#include <tobas_ros2_tools/register.hpp>
#include <tobas_qt_tools/widgets/scroll_area.hpp>
#include <tobas_msgs_adapter/Gps.hpp>
#include <tobas_msgs/msg/rc_input.hpp>
#include <tobas_msgs/msg/pre_arm_check.hpp>

#include "./status.hpp"

namespace gui
{
namespace control_system
{
class StatusViewerWidget : public qt::ScrollArea
{
  Q_OBJECT

  using self = StatusViewerWidget;
  using super = qt::ScrollArea;

public:
  explicit StatusViewerWidget(rclcpp::Node::SharedPtr node);

  void updateNamespace(const std::string& ns);

private:
  const rclcpp::Node::SharedPtr node_;

  StatusWidget* gps_status_;
  StatusWidget* rcin_status_;
  StatusWidget* voltage_status_;
  StatusWidget* attitude_status_;
  StatusWidget* pos_stability_status_;
  StatusWidget* pos_accuracy_status_;
  StatusWidget* rot_accuracy_status_;
  StatusWidget* vel_accuracy_status_;
  StatusWidget* ready_status_;
  StatusWidget* arming_status_;

  ros2::SubscriberPtr<tobas_msgs::Gps> gps_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::RCInput> rcin_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::PreArmCheck> prearm_check_sub_;
  ros2::SubscriberPtr<std_msgs::msg::Bool> arming_sub_;

  void gpsCb(const tobas_msgs::Gps::ConstSharedPtr& gps);
  void rcInputCb(const tobas_msgs::msg::RCInput::ConstSharedPtr& rcin);
  void preArmCheckCb(const tobas_msgs::msg::PreArmCheck::ConstSharedPtr& prearm_check);
  void armingCb(const std_msgs::msg::Bool::ConstSharedPtr& arming);
};
}  // namespace control_system
}  // namespace gui
