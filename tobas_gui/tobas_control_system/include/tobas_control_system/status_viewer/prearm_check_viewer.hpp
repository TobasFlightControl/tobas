#pragma once

#include <tobas_ros2_tools/register.hpp>

#include <tobas_msgs/msg/arming.hpp>
#include <tobas_msgs/msg/pre_arm_check.hpp>

#include "./status.hpp"

namespace gui
{
namespace gcs
{
class PreArmCheckViewerWidget : public QWidget
{
  Q_OBJECT

  using self = PreArmCheckViewerWidget;
  using super = QWidget;

public:
  explicit PreArmCheckViewerWidget(rclcpp::Node::SharedPtr node);

  void reset();
  void updateNamespace(const std::string& ns);

private:
  const rclcpp::Node::SharedPtr node_;

  StatusWidget* node_connection_status_;
  StatusWidget* battery_voltage_status_;
  StatusWidget* cpu_temp_status_;
  StatusWidget* rotor_comm_status_;
  StatusWidget* attitude_level_status_;
  StatusWidget* pos_stability_status_;
  StatusWidget* pos_accuracy_status_;
  StatusWidget* vel_accuracy_status_;
  StatusWidget* atti_accuracy_status_;
  StatusWidget* head_accuracy_status_;
  StatusWidget* ready_status_;

  tobas_msgs::msg::Arming::ConstSharedPtr arming_;

  ros2::SubscriberPtr<tobas_msgs::msg::Arming> arming_sub_;
  ros2::SubscriberPtr<tobas_msgs::msg::PreArmCheck> prearm_check_sub_;

  void armingCb(const tobas_msgs::msg::Arming::ConstSharedPtr& arming);
  void preArmCheckCb(const tobas_msgs::msg::PreArmCheck::ConstSharedPtr& prearm_check);
};
}  // namespace gcs
}  // namespace gui
