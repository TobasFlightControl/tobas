#pragma once

#include <tobas_ros2_tools/register.hpp>
#include <tobas_msgs/msg/arming.hpp>
#include <tobas_msgs/msg/pre_arm_check.hpp>

#include "./status.hpp"

namespace gui
{
namespace control_system
{
class PreArmCheckViewerWidget : public QWidget
{
  Q_OBJECT

  using self = PreArmCheckViewerWidget;
  using super = QWidget;

public:
  explicit PreArmCheckViewerWidget(rclcpp::Node::SharedPtr node);

  void updateNamespace(const std::string& ns);

private:
  const rclcpp::Node::SharedPtr node_;

  StatusWidget* voltage_status_;
  StatusWidget* cpu_status_;
  StatusWidget* rotors_status_;
  StatusWidget* attitude_status_;
  StatusWidget* pos_stability_status_;
  StatusWidget* pos_accuracy_status_;
  StatusWidget* rot_accuracy_status_;
  StatusWidget* vel_accuracy_status_;
  StatusWidget* ready_status_;

  tobas_msgs::msg::Arming::ConstSharedPtr arming_;

  ros2::SubscriberPtr<tobas_msgs::msg::Arming> arming_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::PreArmCheck> prearm_check_sub_;

  void reset();

  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming);
  void preArmCheckCb(const tobas_msgs::msg::PreArmCheck::ConstSharedPtr& prearm_check);
};
}  // namespace control_system
}  // namespace gui
